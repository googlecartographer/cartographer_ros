/*
 * Copyright 2018 The Cartographer Authors
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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_OFFLINE_NODE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_OFFLINE_NODE_H

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer_ros/node_options.h"

namespace cartographer_ros {

using MapBuilderFactory =
    std::function<std::unique_ptr<::cartographer::mapping::MapBuilderInterface>(
        const ::cartographer::mapping::proto::MapBuilderOptions&)>;

void RunOfflineNode(const MapBuilderFactory& map_builder_factory,
                    rclcpp::Node::SharedPtr cartographer_offline_node);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_OFFLINE_NODE_H
