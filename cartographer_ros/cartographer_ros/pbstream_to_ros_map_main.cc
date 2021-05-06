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

#include <map>
#include <string>

#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer_ros/ros_map.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include <rclcpp/rclcpp.hpp>

DEFINE_string(pbstream_filename, "",
              "Filename of a pbstream to draw a map from.");
DEFINE_string(map_filestem, "map", "Stem of the output files.");
DEFINE_double(resolution, 0.05, "Resolution of a grid cell in the drawn map.");

namespace cartographer_ros {
namespace {

void Run(const std::string& pbstream_filename, const std::string& map_filestem,
         const double resolution) {
  ::cartographer::io::ProtoStreamReader reader(pbstream_filename);
  ::cartographer::io::ProtoStreamDeserializer deserializer(&reader);

  LOG(INFO) << "Loading submap slices from serialized data.";
  std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
      submap_slices;
  ::cartographer::mapping::ValueConversionTables conversion_tables;
  ::cartographer::io::DeserializeAndFillSubmapSlices(
      &deserializer, &submap_slices, &conversion_tables);
  CHECK(reader.eof());

  LOG(INFO) << "Generating combined map image from submap slices.";
  auto result =
      ::cartographer::io::PaintSubmapSlices(submap_slices, resolution);

  ::cartographer::io::StreamFileWriter pgm_writer(map_filestem + ".pgm");

  ::cartographer::io::Image image(std::move(result.surface));
  WritePgm(image, resolution, &pgm_writer);

  const Eigen::Vector2d origin(
      -result.origin.x() * resolution,
      (result.origin.y() - image.height()) * resolution);

  ::cartographer::io::StreamFileWriter yaml_writer(map_filestem + ".yaml");
  WriteYaml(resolution, origin, pgm_writer.GetFilename(), &yaml_writer);
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  // Init rclcpp first because gflags reorders command line flags in argv
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing();
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  CHECK(!FLAGS_pbstream_filename.empty()) << "-pbstream_filename is missing.";
  CHECK(!FLAGS_map_filestem.empty()) << "-map_filestem is missing.";

  ::cartographer_ros::Run(FLAGS_pbstream_filename, FLAGS_map_filestem,
                          FLAGS_resolution);
  rclcpp::shutdown();
}
