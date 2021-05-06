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
#include "cartographer_ros_msgs/msg/submap_entry.hpp"
#include "cartographer_ros_msgs/msg/submap_list.hpp"
#include "cartographer_ros_msgs/srv/submap_query.hpp"
#include "gflags/gflags.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <rclcpp/rclcpp.hpp>

DEFINE_double(resolution, 0.05,
              "Resolution of a grid cell in the published occupancy grid.");
DEFINE_double(publish_period_sec, 1.0, "OccupancyGrid publishing period.");
DEFINE_bool(include_frozen_submaps, true,
            "Include frozen submaps in the occupancy grid.");
DEFINE_bool(include_unfrozen_submaps, true,
            "Include unfrozen submaps in the occupancy grid.");
DEFINE_string(occupancy_grid_topic, cartographer_ros::kOccupancyGridTopic,
              "Name of the topic on which the occupancy grid is published.");

namespace cartographer_ros {
namespace {

using ::cartographer::io::PaintSubmapSlicesResult;
using ::cartographer::io::SubmapSlice;
using ::cartographer::mapping::SubmapId;

class Node : public rclcpp::Node
{
 public:
  explicit Node(double resolution, double publish_period_sec);
  ~Node() {}

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

 private:
  void HandleSubmapList(const cartographer_ros_msgs::msg::SubmapList::ConstSharedPtr& msg);
  void DrawAndPublish();

  const double resolution_;

  absl::Mutex mutex_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr callback_group_executor_;
  ::rclcpp::Client<cartographer_ros_msgs::srv::SubmapQuery>::SharedPtr client_ GUARDED_BY(mutex_);
  ::rclcpp::Subscription<cartographer_ros_msgs::msg::SubmapList>::SharedPtr submap_list_subscriber_ GUARDED_BY(mutex_);
  ::rclcpp::Publisher<::nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_ GUARDED_BY(mutex_);
  std::map<SubmapId, SubmapSlice> submap_slices_ GUARDED_BY(mutex_);
  rclcpp::TimerBase::SharedPtr occupancy_grid_publisher_timer_;
  std::string last_frame_id_;
  rclcpp::Time last_timestamp_;
};

Node::Node(const double resolution, const double publish_period_sec)
    : rclcpp::Node("cartographer_occupancy_grid_node"),
      resolution_(resolution)
{
  callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  callback_group_executor_->add_callback_group(callback_group_, this->get_node_base_interface());
  client_ = this->create_client<cartographer_ros_msgs::srv::SubmapQuery>(
        kSubmapQueryServiceName,
        rmw_qos_profile_services_default,
        callback_group_
        );

  occupancy_grid_publisher_ = this->create_publisher<::nav_msgs::msg::OccupancyGrid>(
      kOccupancyGridTopic, rclcpp::QoS(10).transient_local());

  occupancy_grid_publisher_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(int(publish_period_sec * 1000)),
    [this]() {
      DrawAndPublish();
    });

  auto handleSubmapList =
    [this, publish_period_sec](const typename cartographer_ros_msgs::msg::SubmapList::SharedPtr msg) -> void
    {
    absl::MutexLock locker(&mutex_);

    // We do not do any work if nobody listens.
    if (this->count_publishers(kSubmapListTopic) == 0){
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

      auto fetched_textures = cartographer_ros::FetchSubmapTextures(
            id, client_, callback_group_executor_,
            std::chrono::milliseconds(int(publish_period_sec * 1000)));
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
    };

  submap_list_subscriber_ = create_subscription<cartographer_ros_msgs::msg::SubmapList>(
    kSubmapListTopic, rclcpp::QoS(10), handleSubmapList);
}

void Node::DrawAndPublish() {
  absl::MutexLock locker(&mutex_);
  if (submap_slices_.empty() || last_frame_id_.empty()) {
    return;
  }
  auto painted_slices = PaintSubmapSlices(submap_slices_, resolution_);
  std::unique_ptr<nav_msgs::msg::OccupancyGrid> msg_ptr = CreateOccupancyGridMsg(
      painted_slices, resolution_, last_frame_id_, last_timestamp_);
  occupancy_grid_publisher_->publish(*msg_ptr);
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  // Init rclcpp first because gflags reorders command line flags in argv
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing();
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  CHECK(FLAGS_include_frozen_submaps || FLAGS_include_unfrozen_submaps)
      << "Ignoring both frozen and unfrozen submaps makes no sense.";

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  auto node = std::make_shared<cartographer_ros::Node>(FLAGS_resolution, FLAGS_publish_period_sec);

  rclcpp::spin(node);
  ::rclcpp::shutdown();
  return 0;
}
