/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/offline_node.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing();
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  cartographer_ros::ScopedRosLogSink ros_log_sink;

  const cartographer_ros::MapBuilderFactory map_builder_factory = [](
      const ::cartographer::mapping::proto::MapBuilderOptions&
          map_builder_options) {
    return ::cartographer::mapping::CreateMapBuilder(map_builder_options);
  };

  rclcpp::Node::SharedPtr cartographer_offline_node =
      rclcpp::Node::make_shared("cartographer_offline_node");
  cartographer_ros::RunOfflineNode(map_builder_factory, cartographer_offline_node);

  rclcpp::shutdown();
}
