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

#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <string>

#include "absl/memory/memory.h"
#include "cartographer/common/histogram.h"
#include "cartographer_ros/msg_conversion.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "nav_msgs/msg/odometry.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/multi_echo_laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"
#include "urdf/model.h"
#include <tf2/utils.h>

DEFINE_string(bag_filename, "", "Bag to process.");
DEFINE_bool(dump_timing, false,
            "Dump per-sensor timing information in files called "
            "timing_<frame_id>.csv in the current directory.");

namespace cartographer_ros {
namespace {

struct FrameProperties {
  rclcpp::Time last_timestamp;
  std::string topic;
  std::vector<float> time_deltas;
  std::unique_ptr<std::ofstream> timing_file;
  std::string data_type;
};

const double kMinLinearAcceleration = 3.;
const double kMaxLinearAcceleration = 30.;
const double kTimeDeltaSerializationSensorWarning = 0.1;
const double kTimeDeltaSerializationSensorError = 0.5;
const double kMinAverageAcceleration = 9.5;
const double kMaxAverageAcceleration = 10.5;
const double kMaxGapPointsData = 0.1;
const double kMaxGapImuData = 0.05;
const std::set<std::string> kPointDataTypes = {
    std::string(rosidl_generator_traits::data_type<sensor_msgs::msg::PointCloud2>()),
    std::string(rosidl_generator_traits::data_type<sensor_msgs::msg::MultiEchoLaserScan>()),
    std::string(rosidl_generator_traits::data_type<sensor_msgs::msg::LaserScan>())};

std::unique_ptr<std::ofstream> CreateTimingFile(const std::string& frame_id) {
  auto timing_file = absl::make_unique<std::ofstream>(
      std::string("timing_") + frame_id + ".csv", std::ios_base::out);

  (*timing_file)
      << "# Timing information for sensor with frame id: " << frame_id
      << std::endl
      << "# Columns are in order" << std::endl
      << "# - packet index of the packet in the bag, first packet is 1"
      << std::endl
      << "# - timestamp when rosbag wrote the packet, i.e. "
         "rosbag::MessageInstance::getTime().toNSec()"
      << std::endl
      << "# - timestamp when data was acquired, i.e. "
         "message.header.stamp.toNSec()"
      << std::endl
      << "#" << std::endl
      << "# The data can be read in python using" << std::endl
      << "# import numpy" << std::endl
      << "# np.loadtxt(<filename>, dtype='uint64')" << std::endl;

  return timing_file;
}

void CheckImuMessage(const sensor_msgs::msg::Imu::ConstSharedPtr imu_message) {
  auto linear_acceleration = ToEigen(imu_message->linear_acceleration);
  if (std::isnan(linear_acceleration.norm()) ||
      linear_acceleration.norm() < kMinLinearAcceleration ||
      linear_acceleration.norm() > kMaxLinearAcceleration) {
    LOG_FIRST_N(WARNING, 3)
        << "frame_id " << imu_message->header.frame_id << " time "
        << imu_message->header.stamp.nanosec << ": IMU linear acceleration is "
        << linear_acceleration.norm() << " m/s^2,"
        << " expected is [" << kMinLinearAcceleration << ", "
        << kMaxLinearAcceleration << "] m/s^2."
        << " (It should include gravity and be given in m/s^2.)"
        << " linear_acceleration " << linear_acceleration.transpose();
  }
}

bool IsValidPose(const geometry_msgs::msg::Pose& pose) {
  return ToRigid3d(pose).IsValid();
}

void CheckOdometryMessage(nav_msgs::msg::Odometry::ConstSharedPtr message) {
  const auto& pose = message->pose.pose;
  if (!IsValidPose(pose)) {
    LOG_FIRST_N(ERROR, 3) << "frame_id " << message->header.frame_id << " time "
                          << message->header.stamp.nanosec
                          << ": Odometry pose is invalid."
                          << " pose.position.x: " << pose.position.x
                          << " pose.position.y: " << pose.position.y
                          << " pose.position.z: " << pose.position.z
                          << " pose.orientation Yaw: " << tf2::getYaw(pose.orientation)
                             ;
  }
}

void CheckTfMessage(tf2_msgs::msg::TFMessage::ConstSharedPtr message) {
  for (auto transform : message->transforms) {
    if (transform.header.frame_id == "map") {
      LOG_FIRST_N(ERROR, 1)
          << "Input contains transform message from frame_id \""
          << transform.header.frame_id << "\" to child_frame_id \""
          << transform.child_frame_id
          << "\". This is almost always output published by cartographer and "
             "should not appear as input. (Unless you have some complex "
             "remove_frames configuration, this is will not work. Simplest "
             "solution is to record input without cartographer running.)";
    }
  }
}

bool IsPointDataType(const std::string& data_type) {
  return (kPointDataTypes.count(data_type) != 0);
}

class RangeDataChecker {
 public:
  template <typename MessageType>
  void CheckMessage(const MessageType& message) {
    const std::string& frame_id = message.header.frame_id;
    rclcpp::Time current_time_stamp = message.header.stamp;
    RangeChecksum current_checksum;
    cartographer::common::Time time_from, time_to;
    ReadRangeMessage(message, &current_checksum, &time_from, &time_to);
    auto previous_time_to_it = frame_id_to_previous_time_to_.find(frame_id);
    if (previous_time_to_it != frame_id_to_previous_time_to_.end() &&
        previous_time_to_it->second >= time_from) {
      if (previous_time_to_it->second >= time_to) {
        LOG_FIRST_N(ERROR, 3) << "Sensor with frame_id \"" << frame_id
                              << "\" is not sequential in time."
                              << "Previous range message ends at time "
                              << previous_time_to_it->second
                              << ", current one at time " << time_to;
      } else {
        LOG_FIRST_N(WARNING, 3)
            << "Sensor with frame_id \"" << frame_id
            << "\" measurements overlap in time. "
            << "Previous range message, ending at time stamp "
            << previous_time_to_it->second
            << ", must finish before current range message, "
            << "which ranges from " << time_from << " to " << time_to;
      }
      double overlap = cartographer::common::ToSeconds(
          previous_time_to_it->second - time_from);
      auto it = frame_id_to_max_overlap_duration_.find(frame_id);
      if (it == frame_id_to_max_overlap_duration_.end() ||
          overlap > frame_id_to_max_overlap_duration_.at(frame_id)) {
        frame_id_to_max_overlap_duration_[frame_id] = overlap;
      }
    }
    frame_id_to_previous_time_to_[frame_id] = time_to;
    if (current_checksum.first == 0) {
      return;
    }
    auto it = frame_id_to_range_checksum_.find(frame_id);
    if (it != frame_id_to_range_checksum_.end()) {
      RangeChecksum previous_checksum = it->second;
      if (previous_checksum == current_checksum) {
        LOG_FIRST_N(ERROR, 3)
            << "Sensor with frame_id \"" << frame_id
            << "\" sends exactly the same range measurements multiple times. "
            << "Range data at time " << current_time_stamp.seconds()
            << " equals preceding data with " << current_checksum.first
            << " points.";
      }
    }
    frame_id_to_range_checksum_[frame_id] = current_checksum;
  }

  void PrintReport() {
    for (auto& it : frame_id_to_max_overlap_duration_) {
      LOG(WARNING) << "Sensor with frame_id \"" << it.first
                   << "\" range measurements have longest overlap of "
                   << it.second << " s";
    }
  }

 private:
  typedef std::pair<size_t /* num_points */, Eigen::Vector4f /* points_sum */>
      RangeChecksum;

  template <typename MessageType>
  static void ReadRangeMessage(const MessageType& message,
                               RangeChecksum* range_checksum,
                               cartographer::common::Time* from,
                               cartographer::common::Time* to) {
    auto point_cloud_time = ToPointCloudWithIntensities(message);
    const cartographer::sensor::TimedPointCloud& point_cloud =
        std::get<0>(point_cloud_time).points;
    *to = std::get<1>(point_cloud_time);
    *from = *to + cartographer::common::FromSeconds(point_cloud[0].time);
    Eigen::Vector4f points_sum = Eigen::Vector4f::Zero();
    for (const auto& point : point_cloud) {
      points_sum.head<3>() += point.position;
      points_sum[3] += point.time;
    }
    if (point_cloud.size() > 0) {
      double first_point_relative_time = point_cloud[0].time;
      double last_point_relative_time = (*point_cloud.rbegin()).time;
      if (first_point_relative_time > 0) {
        LOG_FIRST_N(ERROR, 1)
            << "Sensor with frame_id \"" << message.header.frame_id
            << "\" has range data that has positive relative time "
            << first_point_relative_time << " s, must negative or zero.";
      }
      if (last_point_relative_time != 0) {
        LOG_FIRST_N(INFO, 1)
            << "Sensor with frame_id \"" << message.header.frame_id
            << "\" has range data whose last point has relative time "
            << last_point_relative_time << " s, should be zero.";
      }
    }
    *range_checksum = {point_cloud.size(), points_sum};
  }

  std::map<std::string, RangeChecksum> frame_id_to_range_checksum_;
  std::map<std::string, cartographer::common::Time>
      frame_id_to_previous_time_to_;
  std::map<std::string, double> frame_id_to_max_overlap_duration_;
};

void Run(const std::string& bag_filename, const bool dump_timing) {
  rosbag2_cpp::Reader bag_reader;
  bag_reader.open(bag_filename);
  rosbag2_storage::BagMetadata bag_metadata = bag_reader.get_metadata();

  auto pcl2_serializer = rclcpp::Serialization<sensor_msgs::msg::PointCloud2>();
  auto multi_echo_laser_scan_serializer = rclcpp::Serialization<sensor_msgs::msg::MultiEchoLaserScan>();
  auto laser_scan_serializer = rclcpp::Serialization<sensor_msgs::msg::LaserScan>();
  auto imu_serializer = rclcpp::Serialization<sensor_msgs::msg::Imu>();
  auto odom_serializer = rclcpp::Serialization<nav_msgs::msg::Odometry>();
  auto tf_serializer = rclcpp::Serialization<tf2_msgs::msg::TFMessage>();

  std::map<std::string, FrameProperties> frame_id_to_properties;
  size_t message_index = 0;
  int num_imu_messages = 0;
  double sum_imu_acceleration = 0.;
  RangeDataChecker range_data_checker;
  while (bag_reader.has_next()) {
    auto message = bag_reader.read_next();
    ++message_index;
    std::string frame_id;
    rclcpp::Time time;
    bool is_data_from_sensor = true;
    std::string topic_type;
    for (auto topic_info : bag_metadata.topics_with_message_count) {
      if (topic_info.topic_metadata.name == message->topic_name){
        if (topic_info.topic_metadata.type == "sensor_msgs/msg/PointCloud2") {
          topic_type = "sensor_msgs/msg/PointCloud2";
          rclcpp::SerializedMessage serialized_msg(*message->serialized_data);
          sensor_msgs::msg::PointCloud2::SharedPtr msg =
              std::make_shared<sensor_msgs::msg::PointCloud2>();
          pcl2_serializer.deserialize_message(&serialized_msg, msg.get());
          time = msg->header.stamp;
          frame_id = msg->header.frame_id;
          range_data_checker.CheckMessage(*msg);
        } else if (topic_info.topic_metadata.type == "sensor_msgs/msg/MultiEchoLaserScan") {
          topic_type = "sensor_msgs/msg/MultiEchoLaserScan";
          rclcpp::SerializedMessage serialized_msg(*message->serialized_data);
          sensor_msgs::msg::MultiEchoLaserScan::SharedPtr msg =
              std::make_shared<sensor_msgs::msg::MultiEchoLaserScan>();
          multi_echo_laser_scan_serializer.deserialize_message(&serialized_msg, msg.get());
          time = msg->header.stamp;
          frame_id = msg->header.frame_id;
          range_data_checker.CheckMessage(*msg);
        } else if (topic_info.topic_metadata.type == "sensor_msgs/msg/LaserScan") {
          topic_type = "sensor_msgs/msg/LaserScan";
          rclcpp::SerializedMessage serialized_msg(*message->serialized_data);
          sensor_msgs::msg::LaserScan::SharedPtr msg =
              std::make_shared<sensor_msgs::msg::LaserScan>();
          laser_scan_serializer.deserialize_message(&serialized_msg, msg.get());
          time = msg->header.stamp;
          frame_id = msg->header.frame_id;
          range_data_checker.CheckMessage(*msg);
        } else if (topic_info.topic_metadata.type == "sensor_msgs/msg/Imu") {
          topic_type = "sensor_msgs/msg/Imu";
          rclcpp::SerializedMessage serialized_msg(*message->serialized_data);
          sensor_msgs::msg::Imu::SharedPtr msg =
              std::make_shared<sensor_msgs::msg::Imu>();
          imu_serializer.deserialize_message(&serialized_msg, msg.get());
          time = msg->header.stamp;
          frame_id = msg->header.frame_id;
          CheckImuMessage(msg);
          num_imu_messages++;
          sum_imu_acceleration += ToEigen(msg->linear_acceleration).norm();
        } else if (topic_info.topic_metadata.type == "nav_msgs/msg/Odometry") {
          topic_type = "nav_msgs/msg/Odometry";
          rclcpp::SerializedMessage serialized_msg(*message->serialized_data);
          nav_msgs::msg::Odometry::SharedPtr msg =
              std::make_shared<nav_msgs::msg::Odometry>();
          odom_serializer.deserialize_message(&serialized_msg, msg.get());
          time = msg->header.stamp;
          frame_id = msg->header.frame_id;
          CheckOdometryMessage(msg);
        } else if (topic_info.topic_metadata.type == "tf2_msgs/msg/TFMessage") {
          rclcpp::SerializedMessage serialized_msg(*message->serialized_data);
          tf2_msgs::msg::TFMessage::SharedPtr msg
              = std::make_shared<tf2_msgs::msg::TFMessage>();;
          tf_serializer.deserialize_message(&serialized_msg, msg.get());
          CheckTfMessage(msg);
          is_data_from_sensor = false;
        } else {
          is_data_from_sensor = false;;
        }
        break;
      }
    }

    if (is_data_from_sensor) {
      bool first_packet = false;
      if (!frame_id_to_properties.count(frame_id)) {
        frame_id_to_properties.emplace(
            frame_id,
            FrameProperties{time, message->topic_name, std::vector<float>(),
                            dump_timing ? CreateTimingFile(frame_id) : nullptr,
                            topic_type});
        first_packet = true;
      }

      auto& entry = frame_id_to_properties.at(frame_id);
      if (!first_packet) {
        const double delta_t_sec = (time - entry.last_timestamp).seconds();
        if (delta_t_sec <= 0) {
          LOG_FIRST_N(ERROR, 3)
              << "Sensor with frame_id \"" << frame_id
              << "\" jumps backwards in time, i.e. timestamps are not strictly "
                 "increasing. Make sure that the bag contains the data for each "
                 "frame_id sorted by header.stamp, i.e. the order in which they "
                 "were acquired from the sensor.";
        }
        entry.time_deltas.push_back(delta_t_sec);
      }

      if (entry.topic != message->topic_name) {
        LOG_FIRST_N(ERROR, 3)
            << "frame_id \"" << frame_id
            << "\" is send on multiple topics. It was seen at least on "
            << entry.topic << " and " << message->topic_name;
      }
      entry.last_timestamp = time;

      if (dump_timing) {
        CHECK(entry.timing_file != nullptr);
        (*entry.timing_file) << message_index << "\t"
                             << message->time_stamp << "\t"
                             << time.nanoseconds() << std::endl;
      }

      double duration_serialization_sensor = (time.nanoseconds() - message->time_stamp)/1e9;
      if (std::abs(duration_serialization_sensor) >
          kTimeDeltaSerializationSensorWarning) {
        std::stringstream stream;
        stream << "frame_id \"" << frame_id << "\" on topic "
               << message->topic_name << " has serialization time "
               << message->time_stamp << " but sensor time " << time.nanoseconds()
               << " differing by " << duration_serialization_sensor << " s.";
        if (std::abs(duration_serialization_sensor) >
            kTimeDeltaSerializationSensorError) {
          LOG_FIRST_N(ERROR, 3) << stream.str();
        } else {
          LOG_FIRST_N(WARNING, 1) << stream.str();
        }
      }
    }
  }

  //end while
  range_data_checker.PrintReport();

  if (num_imu_messages > 0) {
    double average_imu_acceleration = sum_imu_acceleration / num_imu_messages;
    if (std::isnan(average_imu_acceleration) ||
        average_imu_acceleration < kMinAverageAcceleration ||
        average_imu_acceleration > kMaxAverageAcceleration) {
      LOG(ERROR) << "Average IMU linear acceleration is "
                 << average_imu_acceleration << " m/s^2,"
                 << " expected is [" << kMinAverageAcceleration << ", "
                 << kMaxAverageAcceleration
                 << "] m/s^2. Linear acceleration data "
                    "should include gravity and be given in m/s^2.";
    }
  }

  constexpr int kNumBucketsForHistogram = 10;
  for (const auto& entry_pair : frame_id_to_properties) {
    const FrameProperties& frame_properties = entry_pair.second;
    float max_time_delta =
        *std::max_element(frame_properties.time_deltas.begin(),
                          frame_properties.time_deltas.end());
    if (IsPointDataType(frame_properties.data_type) &&
        max_time_delta > kMaxGapPointsData) {
      LOG(ERROR) << "Point data (frame_id: \"" << entry_pair.first
                 << "\") has a large gap, largest is " << max_time_delta
                 << " s, recommended is [0.0005, 0.05] s with no jitter.";
    }
    if (frame_properties.data_type ==
            rosidl_generator_traits::data_type<sensor_msgs::msg::Imu>() &&
        max_time_delta > kMaxGapImuData) {
      LOG(ERROR) << "IMU data (frame_id: \"" << entry_pair.first
                 << "\") has a large gap, largest is " << max_time_delta
                 << " s, recommended is [0.0005, 0.005] s with no jitter.";
    }

    cartographer::common::Histogram histogram;
    for (float time_delta : frame_properties.time_deltas) {
      histogram.Add(time_delta);
    }
    LOG(INFO) << "Time delta histogram for consecutive messages on topic \""
              << frame_properties.topic << "\" (frame_id: \""
              << entry_pair.first << "\"):\n"
              << histogram.ToString(kNumBucketsForHistogram);
  }

  if (dump_timing) {
    for (const auto& entry_pair : frame_id_to_properties) {
      entry_pair.second.timing_file->close();
      CHECK(*entry_pair.second.timing_file)
          << "Could not write timing information for \"" << entry_pair.first
          << "\"";
    }
  }
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

  CHECK(!FLAGS_bag_filename.empty()) << "-bag_filename is missing.";
  ::cartographer_ros::Run(FLAGS_bag_filename, FLAGS_dump_timing);
}
