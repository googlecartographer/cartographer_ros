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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MSG_CONVERSION_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MSG_CONVERSION_H

#include "cartographer/common/time.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/sensor/landmark_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros_msgs/msg/landmark_list.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/multi_echo_laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <rclcpp/rclcpp.hpp>

namespace cartographer_ros {

sensor_msgs::msg::PointCloud2 ToPointCloud2Message(
    int64_t timestamp, const std::string& frame_id,
    const ::cartographer::sensor::TimedPointCloud& point_cloud);

geometry_msgs::msg::Transform ToGeometryMsgTransform(
    const ::cartographer::transform::Rigid3d& rigid3d);

geometry_msgs::msg::Pose ToGeometryMsgPose(
    const ::cartographer::transform::Rigid3d& rigid3d);

geometry_msgs::msg::Point ToGeometryMsgPoint(const Eigen::Vector3d& vector3d);

// Converts ROS message to point cloud. Returns the time when the last point
// was acquired (different from the ROS timestamp). Timing of points is given in
// the fourth component of each point relative to `Time`.
std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::msg::LaserScan& msg);

std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::msg::MultiEchoLaserScan& msg);

std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::msg::PointCloud2& msg);

::cartographer::sensor::LandmarkData ToLandmarkData(
    const cartographer_ros_msgs::msg::LandmarkList& landmark_list);

::cartographer::transform::Rigid3d ToRigid3d(
    const geometry_msgs::msg::TransformStamped& transform);

::cartographer::transform::Rigid3d ToRigid3d(const geometry_msgs::msg::Pose& pose);

Eigen::Vector3d ToEigen(const geometry_msgs::msg::Vector3& vector3);

Eigen::Quaterniond ToEigen(const geometry_msgs::msg::Quaternion& quaternion);

// Converts from WGS84 (latitude, longitude, altitude) to ECEF.
Eigen::Vector3d LatLongAltToEcef(double latitude, double longitude,
                                 double altitude);

// Returns a transform that takes ECEF coordinates from nearby points to a local
// frame that has z pointing upwards.
cartographer::transform::Rigid3d ComputeLocalFrameFromLatLong(double latitude,
                                                              double longitude);

// Points to an occupancy grid message at a specific resolution from painted
// submap slices obtained via ::cartographer::io::PaintSubmapSlices(...).
std::unique_ptr<nav_msgs::msg::OccupancyGrid> CreateOccupancyGridMsg(
    const cartographer::io::PaintSubmapSlicesResult& painted_slices,
    const double resolution, const std::string& frame_id,
    const rclcpp::Time& time);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MSG_CONVERSION_H
