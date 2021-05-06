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

// Publishes a frozen nav_msgs/OccupancyGrid map from serialized submaps.

#include <map>
#include <string>

#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/ros_map.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

DEFINE_string(pbstream_filename, "",
              "Filename of a pbstream to draw a map from.");
DEFINE_string(map_topic, "map", "Name of the published map topic.");
DEFINE_string(map_frame_id, "map", "Frame ID of the published map.");
DEFINE_double(resolution, 0.05, "Resolution of a grid cell in the drawn map.");

namespace cartographer_ros {
namespace {

std::unique_ptr<nav_msgs::msg::OccupancyGrid> LoadOccupancyGridMsg(
    const std::string& pbstream_filename, const double resolution) {
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
  const auto painted_slices =
      ::cartographer::io::PaintSubmapSlices(submap_slices, resolution);
  return CreateOccupancyGridMsg(painted_slices, resolution, FLAGS_map_frame_id,
                                rclcpp::Clock().now());
}

void Run(const std::string& pbstream_filename, const std::string& map_topic,
         const std::string& map_frame_id, const double resolution) {
  rclcpp::Node::SharedPtr cartographer_pbstream_map_publisher_node =
      rclcpp::Node::make_shared("cartographer_pbstream_map_publisher");
  std::unique_ptr<nav_msgs::msg::OccupancyGrid> msg_ptr =
      LoadOccupancyGridMsg(pbstream_filename, resolution);

  auto pub = cartographer_pbstream_map_publisher_node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      map_topic, rclcpp::QoS(1).transient_local());

  LOG(INFO) << "Publishing occupancy grid topic " << map_topic
            << " (frame_id: " << map_frame_id
            << ", resolution:" << std::to_string(resolution) << ").";
  pub->publish(*msg_ptr);
  rclcpp::spin(cartographer_pbstream_map_publisher_node);
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  // Init rclcpp first because gflags reorders command line flags in argv
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing();
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  CHECK(!FLAGS_pbstream_filename.empty()) << "-pbstream_filename is missing.";

  cartographer_ros::ScopedRosLogSink ros_log_sink;

  ::cartographer_ros::Run(FLAGS_pbstream_filename, FLAGS_map_topic,
                          FLAGS_map_frame_id, FLAGS_resolution);
  ::rclcpp::shutdown();
}
