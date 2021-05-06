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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_URDF_READER_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_URDF_READER_H

#include <vector>

#include "cartographer/common/port.h"
#include "tf2_ros/buffer.h"

namespace cartographer_ros {

std::vector<geometry_msgs::msg::TransformStamped> ReadStaticTransformsFromUrdf(
    const std::string& urdf_filename, std::shared_ptr<tf2_ros::Buffer> tf_buffer);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_URDF_READER_H
