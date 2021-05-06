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

#include "cartographer_ros/playable_bag.h"

#include "absl/memory/memory.h"
#include "cartographer_ros/node_constants.h"
#include "glog/logging.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "rosbag2_storage/topic_metadata.hpp"
#include "rosbag2_storage/bag_metadata.hpp"

namespace cartographer_ros {

PlayableBag::PlayableBag(
    const std::string& bag_filename, const int bag_id,
    const rclcpp::Duration buffer_delay,
    FilteringEarlyMessageHandler filtering_early_message_handler)
    : bag_reader_(std::make_unique<rosbag2_cpp::Reader>()),
      finished_(false),
      bag_id_(bag_id),
      bag_filename_(bag_filename),
      message_counter_(0),
      buffer_delay_(buffer_delay),
      filtering_early_message_handler_(
          std::move(filtering_early_message_handler)) {
  LOG(WARNING) << "Opening bag: " << bag_filename;
  bag_reader_->open(bag_filename);
  bag_metadata = bag_reader_->get_metadata();
  duration_in_seconds_ = bag_metadata.duration.count()/1e9;
  LOG(WARNING) << "duration_in_seconds_: " << duration_in_seconds_;
  LOG(WARNING) << "message_count: " << bag_metadata.message_count;
  LOG(WARNING) << "compression_mode: " << bag_metadata.compression_mode;
  LOG(WARNING) << "compression_format: " << bag_metadata.compression_format;
  AdvanceUntilMessageAvailable();
  for (auto topic_info : bag_metadata.topics_with_message_count) {
    topics_.insert(topic_info.topic_metadata.name);
  }
}

rclcpp::Time PlayableBag::PeekMessageTime() const {
  CHECK(IsMessageAvailable());
  return rclcpp::Time(buffered_messages_.front().time_stamp);
}

std::tuple<rclcpp::Time, rclcpp::Time> PlayableBag::GetBeginEndTime() const {
  return std::make_tuple(
    rclcpp::Time(bag_metadata.starting_time.time_since_epoch().count()),
    rclcpp::Time(bag_metadata.starting_time.time_since_epoch().count() + bag_metadata.duration.count()));
}

rosbag2_storage::SerializedBagMessage PlayableBag::GetNextMessage(
    cartographer_ros_msgs::msg::BagfileProgress* progress) {
  CHECK(IsMessageAvailable());
  const rosbag2_storage::SerializedBagMessage msg = buffered_messages_.front();
  buffered_messages_.pop_front();
  AdvanceUntilMessageAvailable();
  double processed_seconds =
      (rclcpp::Time(msg.time_stamp) -
       rclcpp::Time(bag_metadata.starting_time.time_since_epoch().count())).seconds();
  if ((message_counter_ % 10000) == 0) {
    LOG(INFO) << "Processed " << processed_seconds << " of "
              << duration_in_seconds_ << " seconds of bag " << bag_filename_;
  }

  if (progress) {
    progress->current_bagfile_name = bag_filename_;
    progress->current_bagfile_id = bag_id_;
    progress->total_messages = bag_metadata.message_count;
    progress->processed_messages = message_counter_;
    progress->total_seconds = duration_in_seconds_;
    progress->processed_seconds = processed_seconds;
  }

  return msg;
}

bool PlayableBag::IsMessageAvailable() const {
  return !buffered_messages_.empty() &&
         (buffered_messages_.front().time_stamp <
          buffered_messages_.back().time_stamp - buffer_delay_.nanoseconds());
}

int PlayableBag::bag_id() const { return bag_id_; }

void PlayableBag::AdvanceOneMessage() {
  CHECK(!finished_);
  if (!bag_reader_->has_next()) {
    finished_ = true;
    return;
  }
  auto msg = bag_reader_->read_next();
  if (!filtering_early_message_handler_ ||
      filtering_early_message_handler_(msg)) {
    buffered_messages_.push_back(*msg);
  }
  ++message_counter_;
}

void PlayableBag::AdvanceUntilMessageAvailable() {
  if (finished_) {
    return;
  }
  do {
    AdvanceOneMessage();
  } while (!finished_ && !IsMessageAvailable());
}

PlayableBagMultiplexer::PlayableBagMultiplexer(rclcpp::Node::SharedPtr node) {
  node_ = node;
  bag_progress_pub_ = node_->create_publisher<cartographer_ros_msgs::msg::BagfileProgress>(
      "bagfile_progress", 10);
  if (!node_->has_parameter("bagfile_progress_pub_interval")) {
    node_->declare_parameter("bagfile_progress_pub_interval", progress_pub_interval_);
  }
  progress_pub_interval_ =
      node_->get_parameter_or("bagfile_progress_pub_interval", progress_pub_interval_, 10.0);
}

void PlayableBagMultiplexer::AddPlayableBag(PlayableBag playable_bag) {
  for (const auto& topic : playable_bag.topics()) {
    topics_.insert(topic);
  }
  playable_bags_.push_back(std::move(playable_bag));
  CHECK(playable_bags_.back().IsMessageAvailable());
  next_message_queue_.emplace(
      BagMessageItem{playable_bags_.back().PeekMessageTime(),
                     static_cast<int>(playable_bags_.size() - 1)});
  bag_progress_time_map_[playable_bag.bag_id()] = node_->now();
}

bool PlayableBagMultiplexer::IsMessageAvailable() const {
  return !next_message_queue_.empty();
}

std::tuple<rosbag2_storage::SerializedBagMessage, int, std::string, bool> PlayableBagMultiplexer::GetNextMessage() {
  CHECK(IsMessageAvailable());
  const int current_bag_index = next_message_queue_.top().bag_index;
  PlayableBag& current_bag = playable_bags_.at(current_bag_index);
  cartographer_ros_msgs::msg::BagfileProgress progress;
  rosbag2_storage::SerializedBagMessage msg = current_bag.GetNextMessage(&progress);
  const bool publish_progress =
      current_bag.finished() ||
      node_->now() - bag_progress_time_map_[current_bag.bag_id()] >=
          rclcpp::Duration(progress_pub_interval_, 0);
  if (bag_progress_pub_->get_subscription_count() > 0 && publish_progress) {
    progress.total_bagfiles = playable_bags_.size();
    if (current_bag.finished()) {
      progress.processed_seconds = current_bag.duration_in_seconds();
    }
    bag_progress_pub_->publish(progress);
    bag_progress_time_map_[current_bag.bag_id()] = node_->now();
  }
  CHECK_EQ(msg.time_stamp, next_message_queue_.top().message_timestamp.nanoseconds());
  next_message_queue_.pop();
  if (current_bag.IsMessageAvailable()) {
    next_message_queue_.emplace(
        BagMessageItem{current_bag.PeekMessageTime(), current_bag_index});
  }

  std::string topic_type;
  for (auto topic_info : current_bag.bag_metadata.topics_with_message_count) {
    if (topic_info.topic_metadata.name == msg.topic_name){
      topic_type = topic_info.topic_metadata.type;
      break;
    }
  }

  return std::make_tuple(std::move(msg), current_bag.bag_id(), topic_type,
                         !current_bag.IsMessageAvailable());
}

rclcpp::Time PlayableBagMultiplexer::PeekMessageTime() const {
  CHECK(IsMessageAvailable());
  return next_message_queue_.top().message_timestamp;
}

}  // namespace cartographer_ros
