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

#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros_msgs/SubmapQuery.h"

namespace cartographer_ros {

std::unique_ptr<SubmapTexture> FetchSubmapTexture(
    const ::cartographer::mapping::SubmapId& submap_id,
    ros::ServiceClient* client) {
  ::cartographer_ros_msgs::SubmapQuery srv;
  srv.request.trajectory_id = submap_id.trajectory_id;
  srv.request.submap_index = submap_id.submap_index;
  if (!client->call(srv)) {
    return nullptr;
  }
  CHECK(!srv.response.textures.empty()) << "Response contains no textures.";
  // TODO(gaschler): Forward all the textures.
  const auto& texture = srv.response.textures[0];
  std::string compressed_cells(texture.cells.begin(), texture.cells.end());
  std::string cells;
  ::cartographer::common::FastGunzipString(compressed_cells, &cells);
  const int num_pixels = texture.width * texture.height;
  CHECK_EQ(cells.size(), 2 * num_pixels);
  std::vector<char> intensity;
  intensity.reserve(num_pixels);
  std::vector<char> alpha;
  alpha.reserve(num_pixels);
  for (int i = 0; i < texture.height; ++i) {
    for (int j = 0; j < texture.width; ++j) {
      intensity.push_back(cells[(i * texture.width + j) * 2]);
      alpha.push_back(cells[(i * texture.width + j) * 2 + 1]);
    }
  }
  return ::cartographer::common::make_unique<SubmapTexture>(SubmapTexture{
      srv.response.submap_version, intensity, alpha, texture.width,
      texture.height, texture.resolution, ToRigid3d(texture.slice_pose)});
}

}  // namespace cartographer_ros
