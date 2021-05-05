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
#include "absl/synchronization/mutex.h"
#include "cairo/cairo.h"
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
DEFINE_bool(include_frozen_submaps, true,
            "Include frozen submaps in the occupancy grid.");
DEFINE_bool(include_unfrozen_submaps, true,
            "Include unfrozen submaps in the occupancy grid.");
DEFINE_string(occupancy_grid_topic, cartographer_ros::kOccupancyGridTopic,
              "Name of the topic on which the occupancy grid is published.");
DEFINE_bool(trinary_interpretation, false,
              "Publish trinary interpretation of the occupancy grid.");

namespace cartographer_ros {
namespace {

using ::cartographer::io::PaintSubmapSlicesResult;
using ::cartographer::io::SubmapSlice;
using ::cartographer::mapping::SubmapId;

class Node {
 public:
  explicit Node(double resolution, double publish_period_sec, bool use_trinary_interpretation);
  ~Node() {}

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

 private:
  void HandleSubmapList(const cartographer_ros_msgs::SubmapList::ConstPtr& msg);
  void DrawAndPublish(const ::ros::WallTimerEvent& timer_event);

  ::ros::NodeHandle node_handle_;
  const double resolution_;
  const bool use_trinary_interpretation_;

  absl::Mutex mutex_;
  ::ros::ServiceClient client_ GUARDED_BY(mutex_);
  ::ros::Subscriber submap_list_subscriber_ GUARDED_BY(mutex_);
  ::ros::Publisher occupancy_grid_publisher_ GUARDED_BY(mutex_);
  std::map<SubmapId, SubmapSlice> submap_slices_ GUARDED_BY(mutex_);
  ::ros::WallTimer occupancy_grid_publisher_timer_;
  std::string last_frame_id_;
  ros::Time last_timestamp_;
};

Node::Node(const double resolution, const double publish_period_sec, bool use_trinary_interpretation)
    : resolution_(resolution),
      use_trinary_interpretation_(use_trinary_interpretation),
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
              FLAGS_occupancy_grid_topic, kLatestOnlyPublisherQueueSize,
              true /* latched */)),
      occupancy_grid_publisher_timer_(
          node_handle_.createWallTimer(::ros::WallDuration(publish_period_sec),
                                       &Node::DrawAndPublish, this)) {}

void Node::HandleSubmapList(
    const cartographer_ros_msgs::SubmapList::ConstPtr& msg) {
  absl::MutexLock locker(&mutex_);

  // We do not do any work if nobody listens.
  if (occupancy_grid_publisher_.getNumSubscribers() == 0) {
    return;
  }

  // Keep track of submap IDs that don't appear in the message anymore.
  std::set<SubmapId> submap_ids_to_delete;
  for (const auto& pair : submap_slices_) {
    submap_ids_to_delete.insert(pair.first);
  }

  for (const auto& submap_msg : msg->submap) {
    const SubmapId id{submap_msg.trajectory_id, submap_msg.submap_index};
    submap_ids_to_delete.erase(id);
    if ((submap_msg.is_frozen && !FLAGS_include_frozen_submaps) ||
        (!submap_msg.is_frozen && !FLAGS_include_unfrozen_submaps)) {
      continue;
    }
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
    submap_slice.cairo_data.clear();
    submap_slice.surface = ::cartographer::io::DrawTexture(
        fetched_texture->pixels.intensity, fetched_texture->pixels.alpha,
        fetched_texture->width, fetched_texture->height,
        &submap_slice.cairo_data);
  }

  // Delete all submaps that didn't appear in the message.
  for (const auto& id : submap_ids_to_delete) {
    submap_slices_.erase(id);
  }

  last_timestamp_ = msg->header.stamp;
  last_frame_id_ = msg->header.frame_id;
}

void Node::DrawAndPublish(const ::ros::WallTimerEvent& unused_timer_event) {
  absl::MutexLock locker(&mutex_);
  if (submap_slices_.empty() || last_frame_id_.empty()) {
    return;
  }
  auto painted_slices = PaintSubmapSlices(submap_slices_, resolution_);
  std::unique_ptr<nav_msgs::OccupancyGrid> msg_ptr = CreateOccupancyGridMsg(
      painted_slices, resolution_, last_frame_id_, last_timestamp_,
      use_trinary_interpretation_);
  occupancy_grid_publisher_.publish(*msg_ptr);
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(FLAGS_include_frozen_submaps || FLAGS_include_unfrozen_submaps)
      << "Ignoring both frozen and unfrozen submaps makes no sense.";

  ::ros::init(argc, argv, "cartographer_occupancy_grid_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  ::cartographer_ros::Node node(FLAGS_resolution, FLAGS_publish_period_sec, FLAGS_trinary_interpretation);

  ::ros::spin();
  ::ros::shutdown();
}
