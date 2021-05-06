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

#include "cartographer_ros/submap.h"

#include "absl/memory/memory.h"
#include "cartographer/common/port.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros_msgs/msg/status_code.hpp"
#include "cartographer_ros_msgs/srv/submap_query.hpp"

namespace cartographer_ros {

std::unique_ptr<::cartographer::io::SubmapTextures> FetchSubmapTextures(
    const ::cartographer::mapping::SubmapId& submap_id,
    rclcpp::Client<cartographer_ros_msgs::srv::SubmapQuery>::SharedPtr client,
    rclcpp::executors::SingleThreadedExecutor::SharedPtr callback_group_executor,
    const std::chrono::milliseconds timeout)
{
  auto request = std::make_shared<cartographer_ros_msgs::srv::SubmapQuery::Request>();
  request->trajectory_id = submap_id.trajectory_id;
  request->submap_index = submap_id.submap_index;
  auto future_result = client->async_send_request(request);

  if (callback_group_executor->spin_until_future_complete(future_result, timeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return nullptr;
  }
  auto result = future_result.get();

  if (result->status.code != ::cartographer_ros_msgs::msg::StatusCode::OK ||
      result->textures.empty()) {
    return nullptr;
  }

  auto response = absl::make_unique<::cartographer::io::SubmapTextures>();
  response->version = result->submap_version;
  for (const auto& texture : result->textures) {
    const std::string compressed_cells(texture.cells.begin(),
                                       texture.cells.end());
    response->textures.emplace_back(::cartographer::io::SubmapTexture{
        ::cartographer::io::UnpackTextureData(compressed_cells, texture.width,
                                              texture.height),
        texture.width, texture.height, texture.resolution,
        ToRigid3d(texture.slice_pose)});
  }
  return response;
}

}  // namespace cartographer_ros
