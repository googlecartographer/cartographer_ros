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

#include <string>
#include <vector>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/io/points_processor_pipeline_builder.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include <rclcpp/rclcpp.hpp>

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_ASSETS_WRITER_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_ASSETS_WRITER_H

namespace cartographer_ros {

class AssetsWriter {
 public:
  AssetsWriter(const std::string& pose_graph_filename,
               const std::vector<std::string>& bag_filenames,
               const std::string& output_file_prefix);

  // Registers a new PointsProcessor type uniquly identified by 'name' which
  // will be created using 'factory'.
  void RegisterPointsProcessor(
      const std::string& name,
      cartographer::io::PointsProcessorPipelineBuilder::FactoryFunction
          factory);

  // Configures a points processing pipeline and pushes the points from the
  // bag through the pipeline.
  void Run(const std::string& configuration_directory,
           const std::string& configuration_basename,
           const std::string& urdf_filename, bool use_bag_transforms);

  // Creates a FileWriterFactory which creates a FileWriter for storing assets.
  static ::cartographer::io::FileWriterFactory CreateFileWriterFactory(
      const std::string& file_path);

 private:
  std::vector<std::string> bag_filenames_;
  std::vector<::cartographer::mapping::proto::Trajectory> all_trajectories_;
  ::cartographer::mapping::proto::PoseGraph pose_graph_;
  std::unique_ptr<::cartographer::io::PointsProcessorPipelineBuilder>
      point_pipeline_builder_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_ASSETS_WRITER_H
