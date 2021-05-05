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

#include "cartographer_ros/offline_node.h"

#include <errno.h>
#include <string.h>
#ifndef WIN32
#include <sys/resource.h>
#endif
#include <time.h>

#include <chrono>

#include "absl/strings/str_split.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/playable_bag.h"
#include "cartographer_ros/urdf_reader.h"
#include "gflags/gflags.h"
#include "ros/callback_queue.h"
#include "rosgraph_msgs/Clock.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "urdf/model.h"

DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_double(transform_tolerance, 0.0,
              "Time tolerance to apply to transform stamp.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(
    configuration_basenames, "",
    "Comma-separated list of basenames, i.e. not containing any "
    "directory prefix, of the configuration files for each trajectory. "
    "The first configuration file will be used for node options. "
    "If less configuration files are specified than trajectories, the "
    "first file will be used the remaining trajectories.");
DEFINE_string(
    bag_filenames, "",
    "Comma-separated list of bags to process. One bag per trajectory. "
    "Any combination of simultaneous and sequential bags is supported.");
DEFINE_string(urdf_filenames, "",
              "Comma-separated list of one or more URDF files that contain "
              "static links for the sensor configuration(s).");
DEFINE_bool(use_bag_transforms, true,
            "Whether to read, use and republish transforms from bags.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_string(save_state_filename, "",
              "Explicit name of the file to which the serialized state will be "
              "written before shutdown. If left empty, the filename will be "
              "inferred from the first bagfile's name as: "
              "<bag_filenames[0]>.pbstream");
DEFINE_bool(keep_running, false,
            "Keep running the offline node after all messages from the bag "
            "have been processed.");
DEFINE_double(skip_seconds, 0,
              "Optional amount of seconds to skip from the beginning "
              "(i.e. when the earliest bag starts.). ");

namespace cartographer_ros {

constexpr char kClockTopic[] = "clock";
constexpr char kTfStaticTopic[] = "/tf_static";
constexpr char kTfTopic[] = "tf";
constexpr double kClockPublishFrequencySec = 1. / 30.;
constexpr int kSingleThreaded = 1;
// We publish tf messages one second earlier than other messages. Under
// the assumption of higher frequency tf this should ensure that tf can
// always interpolate.
const ::ros::Duration kDelay = ::ros::Duration(1.0);

void RunOfflineNode(const MapBuilderFactory& map_builder_factory) {
  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basenames.empty())
      << "-configuration_basenames is missing.";
  CHECK(!(FLAGS_bag_filenames.empty() && FLAGS_load_state_filename.empty()))
      << "-bag_filenames and -load_state_filename cannot both be unspecified.";
  const std::vector<std::string> bag_filenames =
      absl::StrSplit(FLAGS_bag_filenames, ',', absl::SkipEmpty());
  cartographer_ros::NodeOptions node_options;
  const std::vector<std::string> configuration_basenames =
      absl::StrSplit(FLAGS_configuration_basenames, ',', absl::SkipEmpty());
  std::vector<TrajectoryOptions> bag_trajectory_options(1);
  std::tie(node_options, bag_trajectory_options.at(0)) =
      LoadOptions(FLAGS_configuration_directory, configuration_basenames.at(0));

  for (size_t bag_index = 1; bag_index < bag_filenames.size(); ++bag_index) {
    TrajectoryOptions current_trajectory_options;
    if (bag_index < configuration_basenames.size()) {
      std::tie(std::ignore, current_trajectory_options) = LoadOptions(
          FLAGS_configuration_directory, configuration_basenames.at(bag_index));
    } else {
      current_trajectory_options = bag_trajectory_options.at(0);
    }
    bag_trajectory_options.push_back(current_trajectory_options);
  }
  if (bag_filenames.size() > 0) {
    CHECK_EQ(bag_trajectory_options.size(), bag_filenames.size());
  }

  // Since we preload the transform buffer, we should never have to wait for a
  // transform. When we finish processing the bag, we will simply drop any
  // remaining sensor data that cannot be transformed due to missing transforms.
  node_options.lookup_transform_timeout_sec = 0.;

  auto map_builder = map_builder_factory(node_options.map_builder_options);

  const std::chrono::time_point<std::chrono::steady_clock> start_time =
      std::chrono::steady_clock::now();

  tf2_ros::Buffer tf_buffer;

  std::vector<geometry_msgs::TransformStamped> urdf_transforms;
  const std::vector<std::string> urdf_filenames =
      absl::StrSplit(FLAGS_urdf_filenames, ',', absl::SkipEmpty());
  for (const auto& urdf_filename : urdf_filenames) {
    const auto current_urdf_transforms =
        ReadStaticTransformsFromUrdf(urdf_filename, &tf_buffer);
    urdf_transforms.insert(urdf_transforms.end(),
                           current_urdf_transforms.begin(),
                           current_urdf_transforms.end());
  }

  tf_buffer.setUsingDedicatedThread(true);

  Node node(node_options, std::move(map_builder), &tf_buffer,
            FLAGS_collect_metrics, FLAGS_transform_tolerance);
  if (!FLAGS_load_state_filename.empty()) {
    node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }

  ::ros::Publisher tf_publisher =
      node.node_handle()->advertise<tf2_msgs::TFMessage>(
          kTfTopic, kLatestOnlyPublisherQueueSize);

  ::tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;

  ::ros::Publisher clock_publisher =
      node.node_handle()->advertise<rosgraph_msgs::Clock>(
          kClockTopic, kLatestOnlyPublisherQueueSize);

  if (urdf_transforms.size() > 0) {
    static_tf_broadcaster.sendTransform(urdf_transforms);
  }

  ros::AsyncSpinner async_spinner(kSingleThreaded);
  async_spinner.start();
  rosgraph_msgs::Clock clock;
  auto clock_republish_timer = node.node_handle()->createWallTimer(
      ::ros::WallDuration(kClockPublishFrequencySec),
      [&clock_publisher, &clock](const ::ros::WallTimerEvent&) {
        clock_publisher.publish(clock);
      },
      false /* oneshot */, false /* autostart */);

  std::vector<
      std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
      bag_expected_sensor_ids;
  if (configuration_basenames.size() == 1) {
    const auto current_bag_expected_sensor_ids =
        node.ComputeDefaultSensorIdsForMultipleBags(
            {bag_trajectory_options.front()});
    bag_expected_sensor_ids = {bag_filenames.size(),
                               current_bag_expected_sensor_ids.front()};
  } else {
    bag_expected_sensor_ids =
        node.ComputeDefaultSensorIdsForMultipleBags(bag_trajectory_options);
  }
  CHECK_EQ(bag_expected_sensor_ids.size(), bag_filenames.size());

  std::map<std::pair<int /* bag_index */, std::string>,
           cartographer::mapping::TrajectoryBuilderInterface::SensorId>
      bag_topic_to_sensor_id;
  PlayableBagMultiplexer playable_bag_multiplexer;
  for (size_t current_bag_index = 0; current_bag_index < bag_filenames.size();
       ++current_bag_index) {
    const std::string& bag_filename = bag_filenames.at(current_bag_index);
    if (!::ros::ok()) {
      return;
    }
    for (const auto& expected_sensor_id :
         bag_expected_sensor_ids.at(current_bag_index)) {
      const auto bag_resolved_topic = std::make_pair(
          static_cast<int>(current_bag_index),
          node.node_handle()->resolveName(expected_sensor_id.id));
      if (bag_topic_to_sensor_id.count(bag_resolved_topic) != 0) {
        LOG(ERROR) << "Sensor " << expected_sensor_id.id << " of bag "
                   << current_bag_index << " resolves to topic "
                   << bag_resolved_topic.second << " which is already used by "
                   << " sensor "
                   << bag_topic_to_sensor_id.at(bag_resolved_topic).id;
      }
      bag_topic_to_sensor_id[bag_resolved_topic] = expected_sensor_id;
    }

    playable_bag_multiplexer.AddPlayableBag(PlayableBag(
        bag_filename, current_bag_index, ros::TIME_MIN, ros::TIME_MAX, kDelay,
        // PlayableBag::FilteringEarlyMessageHandler is used to get an early
        // peek at the tf messages in the bag and insert them into 'tf_buffer'.
        // When a message is retrieved by GetNextMessage() further below,
        // we will have already inserted further 'kDelay' seconds worth of
        // transforms into 'tf_buffer' via this lambda.
        [&tf_publisher, &tf_buffer](const rosbag::MessageInstance& msg) {
          if (msg.isType<tf2_msgs::TFMessage>()) {
            if (FLAGS_use_bag_transforms) {
              const auto tf_message = msg.instantiate<tf2_msgs::TFMessage>();
              tf_publisher.publish(tf_message);

              for (const auto& transform : tf_message->transforms) {
                try {
                  // We need to keep 'tf_buffer' small because it becomes very
                  // inefficient otherwise. We make sure that tf_messages are
                  // published before any data messages, so that tf lookups
                  // always work.
                  tf_buffer.setTransform(transform, "unused_authority",
                                         msg.getTopic() == kTfStaticTopic);
                } catch (const tf2::TransformException& ex) {
                  LOG(WARNING) << ex.what();
                }
              }
            }
            // Tell 'PlayableBag' to filter the tf message since there is no
            // further use for it.
            return false;
          } else {
            return true;
          }
        }));
  }

  std::set<std::string> bag_topics;
  std::stringstream bag_topics_string;
  for (const auto& topic : playable_bag_multiplexer.topics()) {
    std::string resolved_topic = node.node_handle()->resolveName(topic, false);
    bag_topics.insert(resolved_topic);
    bag_topics_string << resolved_topic << ",";
  }
  bool print_topics = false;
  for (const auto& entry : bag_topic_to_sensor_id) {
    const std::string& resolved_topic = entry.first.second;
    if (bag_topics.count(resolved_topic) == 0) {
      LOG(WARNING) << "Expected resolved topic \"" << resolved_topic
                   << "\" not found in bag file(s).";
      print_topics = true;
    }
  }
  if (print_topics) {
    LOG(WARNING) << "Available topics in bag file(s) are "
                 << bag_topics_string.str();
  }

  std::unordered_map<int, int> bag_index_to_trajectory_id;
  const ros::Time begin_time =
      // If no bags were loaded, we cannot peek the time of first message.
      playable_bag_multiplexer.IsMessageAvailable()
          ? playable_bag_multiplexer.PeekMessageTime()
          : ros::Time();
  while (playable_bag_multiplexer.IsMessageAvailable()) {
    if (!::ros::ok()) {
      return;
    }

    const auto next_msg_tuple = playable_bag_multiplexer.GetNextMessage();
    const rosbag::MessageInstance& msg = std::get<0>(next_msg_tuple);
    const int bag_index = std::get<1>(next_msg_tuple);
    const bool is_last_message_in_bag = std::get<2>(next_msg_tuple);

    if (msg.getTime() < (begin_time + ros::Duration(FLAGS_skip_seconds))) {
      continue;
    }

    int trajectory_id;
    // Lazily add trajectories only when the first message arrives in order
    // to avoid blocking the sensor queue.
    if (bag_index_to_trajectory_id.count(bag_index) == 0) {
      trajectory_id =
          node.AddOfflineTrajectory(bag_expected_sensor_ids.at(bag_index),
                                    bag_trajectory_options.at(bag_index));
      CHECK(bag_index_to_trajectory_id
                .emplace(std::piecewise_construct,
                         std::forward_as_tuple(bag_index),
                         std::forward_as_tuple(trajectory_id))
                .second);
      LOG(INFO) << "Assigned trajectory " << trajectory_id << " to bag "
                << bag_filenames.at(bag_index);
    } else {
      trajectory_id = bag_index_to_trajectory_id.at(bag_index);
    }

    const auto bag_topic = std::make_pair(
        bag_index,
        node.node_handle()->resolveName(msg.getTopic(), false /* resolve */));
    auto it = bag_topic_to_sensor_id.find(bag_topic);
    if (it != bag_topic_to_sensor_id.end()) {
      const std::string& sensor_id = it->second.id;
      if (msg.isType<sensor_msgs::LaserScan>()) {
        node.HandleLaserScanMessage(trajectory_id, sensor_id,
                                    msg.instantiate<sensor_msgs::LaserScan>());
      }
      if (msg.isType<sensor_msgs::MultiEchoLaserScan>()) {
        node.HandleMultiEchoLaserScanMessage(
            trajectory_id, sensor_id,
            msg.instantiate<sensor_msgs::MultiEchoLaserScan>());
      }
      if (msg.isType<sensor_msgs::PointCloud2>()) {
        node.HandlePointCloud2Message(
            trajectory_id, sensor_id,
            msg.instantiate<sensor_msgs::PointCloud2>());
      }
      if (msg.isType<sensor_msgs::Imu>()) {
        node.HandleImuMessage(trajectory_id, sensor_id,
                              msg.instantiate<sensor_msgs::Imu>());
      }
      if (msg.isType<nav_msgs::Odometry>()) {
        node.HandleOdometryMessage(trajectory_id, sensor_id,
                                   msg.instantiate<nav_msgs::Odometry>());
      }
      if (msg.isType<sensor_msgs::NavSatFix>()) {
        node.HandleNavSatFixMessage(trajectory_id, sensor_id,
                                    msg.instantiate<sensor_msgs::NavSatFix>());
      }
      if (msg.isType<cartographer_ros_msgs::LandmarkList>()) {
        node.HandleLandmarkMessage(
            trajectory_id, sensor_id,
            msg.instantiate<cartographer_ros_msgs::LandmarkList>());
      }
    }
    clock.clock = msg.getTime();
    clock_publisher.publish(clock);

    if (is_last_message_in_bag) {
      node.FinishTrajectory(trajectory_id);
    }
  }

  // Ensure the clock is republished after the bag has been finished, during the
  // final optimization, serialization, and optional indefinite spinning at the
  // end.
  clock_republish_timer.start();
  node.RunFinalOptimization();

  const std::chrono::time_point<std::chrono::steady_clock> end_time =
      std::chrono::steady_clock::now();
  const double wall_clock_seconds =
      std::chrono::duration_cast<std::chrono::duration<double>>(end_time -
                                                                start_time)
          .count();

  LOG(INFO) << "Elapsed wall clock time: " << wall_clock_seconds << " s";
#ifdef __linux__
  timespec cpu_timespec = {};
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cpu_timespec);
  LOG(INFO) << "Elapsed CPU time: "
            << (cpu_timespec.tv_sec + 1e-9 * cpu_timespec.tv_nsec) << " s";
  rusage usage;
  CHECK_EQ(getrusage(RUSAGE_SELF, &usage), 0) << strerror(errno);
  LOG(INFO) << "Peak memory usage: " << usage.ru_maxrss << " KiB";
#endif

  // Serialize unless we have neither a bagfile nor an explicit state filename.
  if (::ros::ok() &&
      !(bag_filenames.empty() && FLAGS_save_state_filename.empty())) {
    const std::string state_output_filename =
        FLAGS_save_state_filename.empty()
            ? absl::StrCat(bag_filenames.front(), ".pbstream")
            : FLAGS_save_state_filename;
    LOG(INFO) << "Writing state to '" << state_output_filename << "'...";
    node.SerializeState(state_output_filename,
                        true /* include_unfinished_submaps */);
  }
  if (FLAGS_keep_running) {
    LOG(INFO) << "Finished processing and waiting for shutdown.";
    ::ros::waitForShutdown();
  }
}

}  // namespace cartographer_ros
