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

#include "cartographer_ros/node_options.h"

#include "glog/logging.h"

namespace cartographer_ros {

NodeOptions CreateNodeOptions(
    ::cartographer::common::LuaParameterDictionary* const
        lua_parameter_dictionary) {
  NodeOptions options;
  options.map_options = CreateMapOptions(lua_parameter_dictionary);
  options.trajectory_options =
      CreateTrajectoryOptions(lua_parameter_dictionary);

  if (options.map_options.map_builder_options.use_trajectory_builder_2d()) {
    // Using point clouds is only supported in 3D.
    CHECK_EQ(options.trajectory_options.num_point_clouds, 0);
  }
  return options;
}

}  // namespace cartographer_ros
