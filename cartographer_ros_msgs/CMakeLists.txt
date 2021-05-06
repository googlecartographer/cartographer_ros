# Copyright 2016 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(cartographer_ros_msgs)

# Default to C++14.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # We don't use add_compile_options with pedantic in message packages
  # because the Python C extensions don't comply with it.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/BagfileProgress.msg"
  "msg/HistogramBucket.msg"
  "msg/LandmarkEntry.msg"
  "msg/LandmarkList.msg"
  "msg/MetricFamily.msg"
  "msg/MetricLabel.msg"
  "msg/Metric.msg"
  "msg/StatusCode.msg"
  "msg/StatusResponse.msg"
  "msg/SubmapEntry.msg"
  "msg/SubmapList.msg"
  "msg/SubmapTexture.msg"
  "msg/TrajectoryStates.msg"
)

set(srv_files
  "srv/FinishTrajectory.srv"
  "srv/GetTrajectoryStates.srv"
  "srv/ReadMetrics.srv"
  "srv/StartTrajectory.srv"
  "srv/TrajectoryQuery.srv"
  "srv/SubmapQuery.srv"
  "srv/WriteState.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES geometry_msgs std_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
