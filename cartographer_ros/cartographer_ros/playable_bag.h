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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_PLAYABLE_BAG_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_PLAYABLE_BAG_H

#include <functional>
#include <queue>

#include "cartographer_ros_msgs/msg/bagfile_progress.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include "tf2_ros/buffer.h"

namespace cartographer_ros {

class PlayableBag {
 public:
  // Handles messages early, i.e. when they are about to enter the buffer.
  // Returns a boolean indicating whether the message should enter the buffer.
  using FilteringEarlyMessageHandler =
      std::function<bool /* forward_message_to_buffer */ (
          std::shared_ptr<rosbag2_storage::SerializedBagMessage>)>;

  PlayableBag(const std::string& bag_filename, int bag_id,
              rclcpp::Duration buffer_delay,
              FilteringEarlyMessageHandler filtering_early_message_handler);

  rclcpp::Time PeekMessageTime() const;
  rosbag2_storage::SerializedBagMessage GetNextMessage(
      cartographer_ros_msgs::msg::BagfileProgress* progress);
  bool IsMessageAvailable() const;
  std::tuple<rclcpp::Time, rclcpp::Time> GetBeginEndTime() const;

  int bag_id() const;
  std::set<std::string> topics() const { return topics_; }
  double duration_in_seconds() const { return duration_in_seconds_; }
  bool finished() const { return finished_; }
  rosbag2_storage::BagMetadata bag_metadata;

 private:
  void AdvanceOneMessage();
  void AdvanceUntilMessageAvailable();

  std::unique_ptr<rosbag2_cpp::Reader> bag_reader_;
  bool finished_;
  const int bag_id_;
  const std::string bag_filename_;
  double duration_in_seconds_;
  int message_counter_;
  std::deque<rosbag2_storage::SerializedBagMessage> buffered_messages_;
  const rclcpp::Duration  buffer_delay_;
  FilteringEarlyMessageHandler filtering_early_message_handler_;
  std::set<std::string> topics_;
};

class PlayableBagMultiplexer {
 public:
  PlayableBagMultiplexer(rclcpp::Node::SharedPtr node);
  void AddPlayableBag(PlayableBag playable_bag);

  // Returns the next message from the multiplexed (merge-sorted) message
  // stream, along with the bag id corresponding to the message, and whether
  // this was the last message in that bag.
  std::tuple<rosbag2_storage::SerializedBagMessage, int, std::string, bool> GetNextMessage();

  bool IsMessageAvailable() const;
  rclcpp::Time PeekMessageTime() const;

  std::set<std::string> topics() const { return topics_; }

 private:
  struct BagMessageItem {
    rclcpp::Time message_timestamp;
    int bag_index;
    struct TimestampIsGreater {
      bool operator()(const BagMessageItem& l, const BagMessageItem& r) {
        return l.message_timestamp > r.message_timestamp;
      }
    };
  };

  rclcpp::Node::SharedPtr node_;
  // Publishes information about the bag-file(s) processing and its progress
  rclcpp::Publisher<cartographer_ros_msgs::msg::BagfileProgress>::SharedPtr bag_progress_pub_;
  // Map between bagfile id and the last time when its progress was published
  std::map<int, rclcpp::Time> bag_progress_time_map_;
  // The time interval of publishing bag-file(s) processing in seconds
  double progress_pub_interval_;

  std::vector<PlayableBag> playable_bags_;
  std::priority_queue<BagMessageItem, std::vector<BagMessageItem>,
                      BagMessageItem::TimestampIsGreater>
      next_message_queue_;
  std::set<std::string> topics_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_PLAYABLE_BAG_H
