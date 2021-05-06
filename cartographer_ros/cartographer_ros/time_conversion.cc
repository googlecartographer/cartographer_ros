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

#include "cartographer_ros/time_conversion.h"

#include "cartographer/common/time.h"
#include <builtin_interfaces/msg/time.hpp>

namespace cartographer_ros {

rclcpp::Time ToRos(::cartographer::common::Time time) {
  int64_t uts_timestamp = ::cartographer::common::ToUniversal(time);
  int64_t ns_since_unix_epoch =
      (uts_timestamp -
       ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds *
           10000000ll) *
      100ll;
  rclcpp::Time ros_time(ns_since_unix_epoch, rcl_clock_type_t::RCL_ROS_TIME);
  return ros_time;
}

// TODO(pedrofernandez): Write test.
::cartographer::common::Time FromRos(const rclcpp::Time& time) {
  // The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
  // exactly 719162 days before the Unix epoch.
  return ::cartographer::common::FromUniversal(
      cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds * 10000000ll +
      (time.nanoseconds() + 50) / 100);  // + 50 to get the rounding correct.
}

}  // namespace cartographer_ros
