/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cmath>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cairo/cairo.h"
#include "cartographer/common/mutex.h"
#include "cartographer/common/port.h"
#include "cartographer/io/image.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/id.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/submap.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "gflags/gflags.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"

DEFINE_double(resolution, 0.05,
              "Resolution of a grid cell in the published occupancy grid.");
DEFINE_double(publish_period_sec, 1.0, "OccupancyGrid publishing period.");

namespace cartographer_ros {
namespace {

using ::cartographer::io::PaintSubmapSlicesResult;
using ::cartographer::io::SubmapSlice;
using ::cartographer::mapping::SubmapId;

class Node {
 public:
  explicit Node(double resolution, double publish_period_sec);
  ~Node() {}

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

 private:
  void HandleSubmapList(const cartographer_ros_msgs::SubmapList::ConstPtr& msg);
  void DrawAndPublish(const ::ros::WallTimerEvent& timer_event);
  void PublishOccupancyGrid(const std::string& frame_id, const ros::Time& time,
                            const Eigen::Array2f& origin,
                            cairo_surface_t* surface);

  ::ros::NodeHandle node_handle_;
  const double resolution_;

  ::cartographer::common::Mutex mutex_;
  ::ros::ServiceClient client_ GUARDED_BY(mutex_);
  ::ros::Subscriber submap_list_subscriber_ GUARDED_BY(mutex_);
  ::ros::Publisher occupancy_grid_publisher_ GUARDED_BY(mutex_);
  std::map<SubmapId, SubmapSlice> submap_slices_ GUARDED_BY(mutex_);
  ::ros::WallTimer occupancy_grid_publisher_timer_;
  std::string last_frame_id_;
  ros::Time last_timestamp_;
};

Node::Node(const double resolution, const double publish_period_sec)
    : resolution_(resolution),
      client_(node_handle_.serviceClient<::cartographer_ros_msgs::SubmapQuery>(
          kSubmapQueryServiceName)),
      submap_list_subscriber_(node_handle_.subscribe(
          kSubmapListTopic, kLatestOnlyPublisherQueueSize,
          boost::function<void(
              const cartographer_ros_msgs::SubmapList::ConstPtr&)>(
              [this](const cartographer_ros_msgs::SubmapList::ConstPtr& msg) {
                HandleSubmapList(msg);
              }))),
      occupancy_grid_publisher_(
          node_handle_.advertise<::nav_msgs::OccupancyGrid>(
              kOccupancyGridTopic, kLatestOnlyPublisherQueueSize,
              true /* latched */)),
      occupancy_grid_publisher_timer_(
          node_handle_.createWallTimer(::ros::WallDuration(publish_period_sec),
                                       &Node::DrawAndPublish, this)) {}

void Node::HandleSubmapList(
    const cartographer_ros_msgs::SubmapList::ConstPtr& msg) {
  ::cartographer::common::MutexLocker locker(&mutex_);

  // We do not do any work if nobody listens.
  if (occupancy_grid_publisher_.getNumSubscribers() == 0) {
    return;
  }
  for (const auto& submap_msg : msg->submap) {
    const SubmapId id{submap_msg.trajectory_id, submap_msg.submap_index};
    SubmapSlice& submap_slice = submap_slices_[id];
    submap_slice.pose = ToRigid3d(submap_msg.pose);
    submap_slice.metadata_version = submap_msg.submap_version;
    if (submap_slice.surface != nullptr &&
        submap_slice.version == submap_msg.submap_version) {
      continue;
    }

    auto fetched_textures =
        ::cartographer_ros::FetchSubmapTextures(id, &client_);
    if (fetched_textures == nullptr) {
      continue;
    }
    CHECK(!fetched_textures->textures.empty());
    submap_slice.version = fetched_textures->version;

    // We use the first texture only. By convention this is the highest
    // resolution texture and that is the one we want to use to construct the
    // map for ROS.
    const auto fetched_texture = fetched_textures->textures.begin();
    submap_slice.width = fetched_texture->width;
    submap_slice.height = fetched_texture->height;
    submap_slice.slice_pose = fetched_texture->slice_pose;
    submap_slice.resolution = fetched_texture->resolution;

    // Properly dealing with a non-common stride would make this code much more
    // complicated. Let's check that it is not needed.
    const int expected_stride = 4 * submap_slice.width;
    // TODO(jihoonl): Refactor here out to remove kCairoFormat dependency
    CHECK_EQ(expected_stride,
             cairo_format_stride_for_width(
                 PaintSubmapSlicesResult::kCairoFormat, submap_slice.width));
    submap_slice.cairo_data.clear();
    for (size_t i = 0; i < fetched_texture->intensity.size(); ++i) {
      // We use the red channel to track intensity information. The green
      // channel we use to track if a cell was ever observed.
      const uint8_t intensity = fetched_texture->intensity.at(i);
      const uint8_t alpha = fetched_texture->alpha.at(i);
      const uint8_t observed = (intensity == 0 && alpha == 0) ? 0 : 255;
      submap_slice.cairo_data.push_back((alpha << 24) | (intensity << 16) |
                                        (observed << 8) | 0);
    }

    submap_slice.surface = ::cartographer::io::MakeUniqueCairoSurfacePtr(
        cairo_image_surface_create_for_data(
            reinterpret_cast<unsigned char*>(submap_slice.cairo_data.data()),
            PaintSubmapSlicesResult::kCairoFormat, submap_slice.width,
            submap_slice.height, expected_stride));
    CHECK_EQ(cairo_surface_status(submap_slice.surface.get()),
             CAIRO_STATUS_SUCCESS)
        << cairo_status_to_string(
            cairo_surface_status(submap_slice.surface.get()));
  }
  last_timestamp_ = msg->header.stamp;
  last_frame_id_ = msg->header.frame_id;
}

void Node::DrawAndPublish(const ::ros::WallTimerEvent& unused_timer_event) {
  if (submap_slices_.empty() || last_frame_id_.empty()) {
    return;
  }

  ::cartographer::common::MutexLocker locker(&mutex_);
  auto painted_slices = PaintSubmapSlices(&submap_slices_, resolution_);
  PublishOccupancyGrid(last_frame_id_, last_timestamp_, painted_slices.origin,
                       painted_slices.surface.get());
}

void Node::PublishOccupancyGrid(const std::string& frame_id,
                                const ros::Time& time,
                                const Eigen::Array2f& origin,
                                cairo_surface_t* surface) {
  nav_msgs::OccupancyGrid occupancy_grid;
  const int width = cairo_image_surface_get_width(surface);
  const int height = cairo_image_surface_get_height(surface);
  occupancy_grid.header.stamp = time;
  occupancy_grid.header.frame_id = frame_id;
  occupancy_grid.info.map_load_time = time;
  occupancy_grid.info.resolution = resolution_;
  occupancy_grid.info.width = width;
  occupancy_grid.info.height = height;
  occupancy_grid.info.origin.position.x = -origin.x() * resolution_;
  occupancy_grid.info.origin.position.y =
      (-height + origin.y()) * resolution_;
  occupancy_grid.info.origin.position.z = 0.;
  occupancy_grid.info.origin.orientation.w = 1.;
  occupancy_grid.info.origin.orientation.x = 0.;
  occupancy_grid.info.origin.orientation.y = 0.;
  occupancy_grid.info.origin.orientation.z = 0.;

  const uint32_t* pixel_data = reinterpret_cast<uint32_t*>(cairo_image_surface_get_data(surface));
  occupancy_grid.data.reserve(width * height);
  for (int y = height - 1; y >= 0; --y) {
    for (int x = 0; x < width; ++x) {
      const uint32_t packed = pixel_data[y * width + x];
      const unsigned char color = packed >> 16;
      const unsigned char observed = packed >> 8;
      const int value = observed == 0 ? -1 : ::cartographer::common::RoundToInt(
                                                 (1. - color / 255.) * 100.);
      CHECK_LE(-1, value);
      CHECK_GE(100, value);
      occupancy_grid.data.push_back(value);
    }
  }
  occupancy_grid_publisher_.publish(occupancy_grid);
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  ::ros::init(argc, argv, "cartographer_occupancy_grid_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  ::cartographer_ros::Node node(FLAGS_resolution, FLAGS_publish_period_sec);

  ::ros::spin();
  ::ros::shutdown();
}
