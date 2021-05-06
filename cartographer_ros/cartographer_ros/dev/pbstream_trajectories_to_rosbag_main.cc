/*
 * Copyright 2019 The Cartographer Authors
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

#include <iostream>
#include <string>
#include <vector>

#include "absl/strings/str_cat.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/time_conversion.h"
#include "geometry_msgs/TransformStamped.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "rosbag/bag.h"
#include "tf2_msgs/TFMessage.h"

DEFINE_string(input, "", "pbstream file to process");
DEFINE_string(output, "", "Bag file to write to.");
DEFINE_string(parent_frame, "map", "Frame id to use as parent frame.");

namespace cartographer_ros {
namespace {

geometry_msgs::msg::TransformStamped ToTransformStamped(
    int64_t timestamp_uts, const std::string& parent_frame_id,
    const std::string& child_frame_id,
    const cartographer::transform::proto::Rigid3d& parent_T_child) {
  static int64_t seq = 0;
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.seq = ++seq;
  transform_stamped.header.frame_id = parent_frame_id;
  transform_stamped.header.stamp = cartographer_ros::ToRos(
      ::cartographer::common::FromUniversal(timestamp_uts));
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.transform = cartographer_ros::ToGeometryMsgTransform(
      ::cartographer::transform::ToRigid3(parent_T_child));
  return transform_stamped;
}

void pbstream_trajectories_to_bag(const std::string& pbstream_filename,
                                  const std::string& output_bag_filename,
                                  const std::string& parent_frame_id) {
  const auto pose_graph =
      cartographer::io::DeserializePoseGraphFromFile(FLAGS_input);

  rosbag::Bag bag(output_bag_filename, rosbag::bagmode::Write);
  for (const auto trajectory : pose_graph.trajectory()) {
    const auto child_frame_id =
        absl::StrCat("trajectory_", trajectory.trajectory_id());
    LOG(INFO)
        << "Writing tf and geometry_msgs/TransformStamped for trajectory id "
        << trajectory.trajectory_id() << " with " << trajectory.node_size()
        << " nodes.";
    for (const auto& node : trajectory.node()) {
      tf2_msgs::TFMessage tf_msg;
      geometry_msgs::msg::TransformStamped transform_stamped = ToTransformStamped(
          node.timestamp(), parent_frame_id, child_frame_id, node.pose());
      tf_msg.transforms.push_back(transform_stamped);
      bag.write(child_frame_id, transform_stamped.header.stamp,
                transform_stamped);
      bag.write("/tf", transform_stamped.header.stamp, tf_msg);
    }
  }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char* argv[]) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::SetUsageMessage(
      "\n\n"
      "Extracts all trajectories from the pbstream and creates a bag file with "
      "the trajectory poses stored in /tf.\nAdditionally, each trajectory is "
      "also written separately to a geometry_msgs/TransformStamped topic named "
      "after the TF child_frame_id of the trajectory.\n For each trajectory, "
      "the tool will write transforms with the tf parent_frame_id set "
      "according to the `parent_frame` commandline flag and child_frame_id to "
      "`trajectory_i`, with `i` corresponding to the `trajectory_id`.");
  google::ParseCommandLineFlags(&argc, &argv, true);
  CHECK(!FLAGS_input.empty()) << "-input pbstream is missing.";
  CHECK(!FLAGS_output.empty()) << "-output is missing.";

  cartographer_ros::pbstream_trajectories_to_bag(FLAGS_input, FLAGS_output,
                                                 FLAGS_parent_frame);
  return 0;
}
