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

#include "cartographer_ros/assets_writer.h"

#include <algorithm>
#include <fstream>
#include <iostream>

#include "absl/memory/memory.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/math.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/io/points_processor_pipeline_builder.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/transform_interpolation_buffer.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/ros_map_writing_points_processor.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros/urdf_reader.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "builtin_interfaces/msg/time.hpp"
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"
#include "urdf/model.h"

namespace cartographer_ros {
namespace {

constexpr char kTfStaticTopic[] = "/tf_static";
namespace carto = ::cartographer;

std::unique_ptr<carto::io::PointsProcessorPipelineBuilder>
CreatePipelineBuilder(
    const std::vector<carto::mapping::proto::Trajectory>& trajectories,
    const std::string file_prefix) {
  const auto file_writer_factory =
      AssetsWriter::CreateFileWriterFactory(file_prefix);
  auto builder = absl::make_unique<carto::io::PointsProcessorPipelineBuilder>();
  carto::io::RegisterBuiltInPointsProcessors(trajectories, file_writer_factory,
                                             builder.get());
  builder->Register(RosMapWritingPointsProcessor::kConfigurationFileActionName,
                    [file_writer_factory](
                        carto::common::LuaParameterDictionary* const dictionary,
                        carto::io::PointsProcessor* const next)
                        -> std::unique_ptr<carto::io::PointsProcessor> {
                      return RosMapWritingPointsProcessor::FromDictionary(
                          file_writer_factory, dictionary, next);
                    });
  return builder;
}

std::unique_ptr<carto::common::LuaParameterDictionary> LoadLuaDictionary(
    const std::string& configuration_directory,
    const std::string& configuration_basename) {
  auto file_resolver =
      absl::make_unique<carto::common::ConfigurationFileResolver>(
          std::vector<std::string>{configuration_directory});

  const std::string code =
      file_resolver->GetFileContentOrDie(configuration_basename);
  auto lua_parameter_dictionary =
      absl::make_unique<carto::common::LuaParameterDictionary>(
          code, std::move(file_resolver));
  return lua_parameter_dictionary;
}

template <typename T>
std::unique_ptr<carto::io::PointsBatch> HandleMessage(
    const T& message, const std::string& tracking_frame,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const carto::transform::TransformInterpolationBuffer&
        transform_interpolation_buffer) {
  const carto::common::Time start_time = FromRos(message->header.stamp);

  auto points_batch = absl::make_unique<carto::io::PointsBatch>();
  points_batch->start_time = start_time;
  points_batch->frame_id = message->header.frame_id;

  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time point_cloud_time;
  std::tie(point_cloud, point_cloud_time) =
      ToPointCloudWithIntensities(*message);
  CHECK_EQ(point_cloud.intensities.size(), point_cloud.points.size());

  for (size_t i = 0; i < point_cloud.points.size(); ++i) {
    const carto::common::Time time =
        point_cloud_time +
        carto::common::FromSeconds(point_cloud.points[i].time);
    if (!transform_interpolation_buffer.Has(time)) {
      continue;
    }
    const carto::transform::Rigid3d tracking_to_map =
        transform_interpolation_buffer.Lookup(time);
    const carto::transform::Rigid3d sensor_to_tracking =
        ToRigid3d(tf_buffer->lookupTransform(
            tracking_frame, message->header.frame_id, ToRos(time)));
    const carto::transform::Rigid3f sensor_to_map =
        (tracking_to_map * sensor_to_tracking).cast<float>();
    points_batch->points.push_back(
        sensor_to_map *
        carto::sensor::ToRangefinderPoint(point_cloud.points[i]));
    points_batch->intensities.push_back(point_cloud.intensities[i]);
    // We use the last transform for the origin, which is approximately correct.
    points_batch->origin = sensor_to_map * Eigen::Vector3f::Zero();
  }
  if (points_batch->points.empty()) {
    return nullptr;
  }
  return points_batch;
}

}  // namespace

AssetsWriter::AssetsWriter(const std::string& pose_graph_filename,
                           const std::vector<std::string>& bag_filenames,
                           const std::string& output_file_prefix)
    : bag_filenames_(bag_filenames),
      pose_graph_(
          carto::io::DeserializePoseGraphFromFile(pose_graph_filename)) {
  CHECK_EQ(pose_graph_.trajectory_size(), bag_filenames_.size())
      << "Pose graphs contains " << pose_graph_.trajectory_size()
      << " trajectories while " << bag_filenames_.size()
      << " bags were provided. This tool requires one bag for each "
         "trajectory in the same order as the correponding trajectories in the "
         "pose graph proto.";

  // This vector must outlive the pipeline.
  all_trajectories_ = std::vector<::cartographer::mapping::proto::Trajectory>(
      pose_graph_.trajectory().begin(), pose_graph_.trajectory().end());

  const std::string file_prefix = !output_file_prefix.empty()
                                      ? output_file_prefix
                                      : bag_filenames_.front() + "_";
  point_pipeline_builder_ =
      CreatePipelineBuilder(all_trajectories_, file_prefix);
}

void AssetsWriter::RegisterPointsProcessor(
    const std::string& name,
    cartographer::io::PointsProcessorPipelineBuilder::FactoryFunction factory) {
  point_pipeline_builder_->Register(name, factory);
}

void AssetsWriter::Run(const std::string& configuration_directory,
                       const std::string& configuration_basename,
                       const std::string& urdf_filename,
                       const bool use_bag_transforms) {
  const auto lua_parameter_dictionary =
      LoadLuaDictionary(configuration_directory, configuration_basename);

  std::vector<std::unique_ptr<carto::io::PointsProcessor>> pipeline =
      point_pipeline_builder_->CreatePipeline(
          lua_parameter_dictionary->GetDictionary("pipeline").get());
  const std::string tracking_frame =
      lua_parameter_dictionary->GetString("tracking_frame");

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  do {
    for (size_t trajectory_id = 0; trajectory_id < bag_filenames_.size();
         ++trajectory_id) {
      const carto::mapping::proto::Trajectory& trajectory_proto =
          pose_graph_.trajectory(trajectory_id);
      const std::string& bag_filename = bag_filenames_[trajectory_id];
      LOG(INFO) << "Processing " << bag_filename << "...";
      if (trajectory_proto.node_size() == 0) {
        continue;
      }

      std::shared_ptr<tf2_ros::Buffer> tf_buffer = std::make_shared<tf2_ros::Buffer>(clock);
      tf_buffer->setUsingDedicatedThread(true);

      if (!urdf_filename.empty()) {
        ReadStaticTransformsFromUrdf(urdf_filename, tf_buffer);
      }

      const carto::transform::TransformInterpolationBuffer
          transform_interpolation_buffer(trajectory_proto);
      rosbag2_cpp::Reader bag_reader;
      bag_reader.open(bag_filename);
      rosbag2_storage::BagMetadata bag_metadata = bag_reader.get_metadata();
      const rclcpp::Time begin_time(bag_metadata.starting_time.time_since_epoch().count());

      // We need to keep 'tf_buffer' small because it becomes very inefficient
      // otherwise. We make sure that tf_messages are published before any data
      // messages, so that tf lookups always work.
      std::deque<rosbag2_storage::SerializedBagMessage> delayed_messages;
      // We publish tf messages one second earlier than other messages. Under
      // the assumption of higher frequency tf this should ensure that tf can
      // always interpolate.
      const rclcpp::Duration kDelay(1.0,0.0);
      auto serializer = rclcpp::Serialization<tf2_msgs::msg::TFMessage>();
      auto laser_scan_serializer = rclcpp::Serialization<sensor_msgs::msg::LaserScan>();
      auto multi_echo_laser_scan_serializer = rclcpp::Serialization<sensor_msgs::msg::MultiEchoLaserScan>();
      auto pcl2_serializer = rclcpp::Serialization<sensor_msgs::msg::PointCloud2>();
      while (bag_reader.has_next()) {
        auto message = bag_reader.read_next();
        if (use_bag_transforms && (message->topic_name  == kTfStaticTopic || message->topic_name  == "/tf")) {
          tf2_msgs::msg::TFMessage tf_message;
          rclcpp::SerializedMessage serialized_msg(*message->serialized_data);
          serializer.deserialize_message(&serialized_msg, &tf_message);
          for (const auto& transform : tf_message.transforms) {
            try {
              tf_buffer->setTransform(transform, "unused_authority",
                                     message->topic_name == kTfStaticTopic);
            } catch (const tf2::TransformException& ex) {
              LOG(WARNING) << ex.what();
            }
          }
        }

        while (!delayed_messages.empty() && delayed_messages.front().time_stamp <
                                                message->time_stamp - kDelay.nanoseconds()) {
          const auto delayed_message =
              delayed_messages.front();

          std::unique_ptr<carto::io::PointsBatch> points_batch;

          for (auto topic_info : bag_metadata.topics_with_message_count) {
            if (topic_info.topic_metadata.name == delayed_message.topic_name){
              rclcpp::SerializedMessage serialized_msg(*delayed_message.serialized_data);
              if (topic_info.topic_metadata.type == "sensor_msgs/msg/LaserScan") {
                sensor_msgs::msg::LaserScan::SharedPtr laser_scan_msg =
                    std::make_shared<sensor_msgs::msg::LaserScan>();
                laser_scan_serializer.deserialize_message(&serialized_msg, laser_scan_msg.get());
                points_batch = HandleMessage(
                    laser_scan_msg,
                    tracking_frame, tf_buffer, transform_interpolation_buffer);
              } else if (topic_info.topic_metadata.type == "sensor_msgs/msg/MultiEchoLaserScan") {
                sensor_msgs::msg::MultiEchoLaserScan::SharedPtr multi_echo_laser_scan_msg =
                    std::make_shared<sensor_msgs::msg::MultiEchoLaserScan>();
                multi_echo_laser_scan_serializer.deserialize_message(&serialized_msg, &multi_echo_laser_scan_msg);
                points_batch = HandleMessage(
                    multi_echo_laser_scan_msg,
                    tracking_frame, tf_buffer, transform_interpolation_buffer);
              }
              else if (topic_info.topic_metadata.type == "sensor_msgs/msg/PointCloud2") {
                sensor_msgs::msg::PointCloud2::SharedPtr pcl2_scan_msg =
                    std::make_shared<sensor_msgs::msg::PointCloud2>();
                pcl2_serializer.deserialize_message(&serialized_msg, &pcl2_scan_msg);
                points_batch = HandleMessage(
                    pcl2_scan_msg,
                    tracking_frame, tf_buffer, transform_interpolation_buffer);
              }
              if (points_batch != nullptr) {
                points_batch->trajectory_id = trajectory_id;
                pipeline.back()->Process(std::move(points_batch));
              }
              delayed_messages.pop_front();
              break;
            }
          }
        }
        delayed_messages.push_back(*message);
        LOG_EVERY_N(INFO, 10000)
            << "Processed " << (message->time_stamp - begin_time.nanoseconds())/1e9
            << " of " << bag_metadata.duration.count()/1e9 << " bag time seconds...";
      }
    }
  } while (pipeline.back()->Flush() ==
           carto::io::PointsProcessor::FlushResult::kRestartStream);
}

::cartographer::io::FileWriterFactory AssetsWriter::CreateFileWriterFactory(
    const std::string& file_path) {
  const auto file_writer_factory = [file_path](const std::string& filename) {
    return absl::make_unique<carto::io::StreamFileWriter>(file_path + filename);
  };
  return file_writer_factory;
}

}  // namespace cartographer_ros
