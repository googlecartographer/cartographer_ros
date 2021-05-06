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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SUBMAP_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SUBMAP_H

#include <memory>
#include <string>
#include <vector>

#include "cartographer/io/image.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/id.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros_msgs/srv/submap_query.hpp"
#include <rclcpp/rclcpp.hpp>

namespace cartographer_ros {

// Fetch 'submap_id' using the 'client' and returning the response or 'nullptr'
// on error.
std::unique_ptr<::cartographer::io::SubmapTextures> FetchSubmapTextures(
    const ::cartographer::mapping::SubmapId& submap_id,
    rclcpp::Client<cartographer_ros_msgs::srv::SubmapQuery>::SharedPtr client,
    rclcpp::executors::SingleThreadedExecutor::SharedPtr callback_group_executor,
    const std::chrono::milliseconds timeout);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SUBMAP_H
