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

#include "cartographer_ros/msg_conversion.h"

#include <cmath>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/transform/proto/transform.pb.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/time_conversion.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "glog/logging.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "builtin_interfaces/msg/time.hpp"
//#include "ros/serialization.h"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/multi_echo_laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace {

// Sizes of PCL point types have to be 4n floats for alignment, as described in
// http://pointclouds.org/documentation/tutorials/adding_custom_ptype.php
struct PointXYZT {
  float x;
  float y;
  float z;
  float time;
};

struct PointXYZIT {
  PCL_ADD_POINT4D;
  float intensity;
  float time;
  float unused_padding[2];
};

}  // namespace

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZT, (float, x, x)(float, y, y)(float, z, z)(float, time, time))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                            intensity)(float, time, time))

namespace cartographer_ros {
namespace {

// The ros::sensor_msgs::msg::PointCloud2 binary data contains 4 floats for each
// point. The last one must be this value or RViz is not showing the point cloud
// properly.
constexpr float kPointCloudComponentFourMagic = 1.;

using ::cartographer::sensor::LandmarkData;
using ::cartographer::sensor::LandmarkObservation;
using ::cartographer::sensor::PointCloudWithIntensities;
using ::cartographer::transform::Rigid3d;
using ::cartographer_ros_msgs::msg::LandmarkEntry;
using ::cartographer_ros_msgs::msg::LandmarkList;

sensor_msgs::msg::PointCloud2 PreparePointCloud2Message(const int64_t timestamp,
                                                   const std::string& frame_id,
                                                   const int num_points) {
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp = ToRos(::cartographer::common::FromUniversal(timestamp));
  msg.header.frame_id = frame_id;
  msg.height = 1;
  msg.width = num_points;
  msg.fields.resize(3);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[2].count = 1;
  msg.is_bigendian = false;
  msg.point_step = 16;
  msg.row_step = 16 * msg.width;
  msg.is_dense = true;
  msg.data.resize(16 * num_points);
  return msg;
}

// For sensor_msgs::msg::LaserScan.
bool HasEcho(float) { return true; }

float GetFirstEcho(float range) { return range; }

// For sensor_msgs::msg::MultiEchoLaserScan.
bool HasEcho(const sensor_msgs::msg::LaserEcho& echo) {
  return !echo.echoes.empty();
}

float GetFirstEcho(const sensor_msgs::msg::LaserEcho& echo) {
  return echo.echoes[0];
}

// For sensor_msgs::msg::LaserScan and sensor_msgs::msg::MultiEchoLaserScan.
template <typename LaserMessageType>
std::tuple<PointCloudWithIntensities, ::cartographer::common::Time>
LaserScanToPointCloudWithIntensities(const LaserMessageType& msg) {
  CHECK_GE(msg.range_min, 0.f);
  CHECK_GE(msg.range_max, msg.range_min);
  if (msg.angle_increment > 0.f) {
    CHECK_GT(msg.angle_max, msg.angle_min);
  } else {
    CHECK_GT(msg.angle_min, msg.angle_max);
  }
  PointCloudWithIntensities point_cloud;
  float angle = msg.angle_min;
  for (size_t i = 0; i < msg.ranges.size(); ++i) {
    const auto& echoes = msg.ranges[i];
    if (HasEcho(echoes)) {
      const float first_echo = GetFirstEcho(echoes);
      if (msg.range_min <= first_echo && first_echo <= msg.range_max) {
        const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
        const cartographer::sensor::TimedRangefinderPoint point{
            rotation * (first_echo * Eigen::Vector3f::UnitX()),
            i * msg.time_increment};
        point_cloud.points.push_back(point);
        if (msg.intensities.size() > 0) {
          CHECK_EQ(msg.intensities.size(), msg.ranges.size());
          const auto& echo_intensities = msg.intensities[i];
          CHECK(HasEcho(echo_intensities));
          point_cloud.intensities.push_back(GetFirstEcho(echo_intensities));
        } else {
          point_cloud.intensities.push_back(0.f);
        }
      }
    }
    angle += msg.angle_increment;
  }
  ::cartographer::common::Time timestamp = FromRos(msg.header.stamp);
  if (!point_cloud.points.empty()) {
    const double duration = point_cloud.points.back().time;
    timestamp += cartographer::common::FromSeconds(duration);
    for (auto& point : point_cloud.points) {
      point.time -= duration;
    }
  }
  return std::make_tuple(point_cloud, timestamp);
}

bool PointCloud2HasField(const sensor_msgs::msg::PointCloud2& pc2,
                         const std::string& field_name) {
  for (const auto& field : pc2.fields) {
    if (field.name == field_name) {
      return true;
    }
  }
  return false;
}

}  // namespace

sensor_msgs::msg::PointCloud2 ToPointCloud2Message(
    const int64_t timestamp, const std::string& frame_id,
    const ::cartographer::sensor::TimedPointCloud& point_cloud) {
  auto msg = PreparePointCloud2Message(timestamp, frame_id, point_cloud.size());
  size_t offset = 0;
  float * const data = reinterpret_cast<float*>(&msg.data[0]);
  for (const auto& point : point_cloud) {
    data[offset++] = point.position.x();
    data[offset++] = point.position.y();
    data[offset++] = point.position.z();
    data[offset++] = kPointCloudComponentFourMagic;
  }
  return msg;
}

std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::msg::LaserScan& msg) {
  return LaserScanToPointCloudWithIntensities(msg);
}

std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::msg::MultiEchoLaserScan& msg) {
  return LaserScanToPointCloudWithIntensities(msg);
}

std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::msg::PointCloud2& msg) {
  PointCloudWithIntensities point_cloud;
  // We check for intensity field here to avoid run-time warnings if we pass in
  // a PointCloud2 without intensity.
  if (PointCloud2HasField(msg, "intensity")) {
    if (PointCloud2HasField(msg, "time")) {
      pcl::PointCloud<PointXYZIT> pcl_point_cloud;
      pcl::fromROSMsg(msg, pcl_point_cloud);
      point_cloud.points.reserve(pcl_point_cloud.size());
      point_cloud.intensities.reserve(pcl_point_cloud.size());
      for (const auto& point : pcl_point_cloud) {
        point_cloud.points.push_back(
            {Eigen::Vector3f{point.x, point.y, point.z}, point.time});
        point_cloud.intensities.push_back(point.intensity);
      }
    } else {
      pcl::PointCloud<pcl::PointXYZI> pcl_point_cloud;
      pcl::fromROSMsg(msg, pcl_point_cloud);
      point_cloud.points.reserve(pcl_point_cloud.size());
      point_cloud.intensities.reserve(pcl_point_cloud.size());
      for (const auto& point : pcl_point_cloud) {
        point_cloud.points.push_back(
            {Eigen::Vector3f{point.x, point.y, point.z}, 0.f});
        point_cloud.intensities.push_back(point.intensity);
      }
    }
  } else {
    // If we don't have an intensity field, just copy XYZ and fill in 1.0f.
    if (PointCloud2HasField(msg, "time")) {
      pcl::PointCloud<PointXYZT> pcl_point_cloud;
      pcl::fromROSMsg(msg, pcl_point_cloud);
      point_cloud.points.reserve(pcl_point_cloud.size());
      point_cloud.intensities.reserve(pcl_point_cloud.size());
      for (const auto& point : pcl_point_cloud) {
        point_cloud.points.push_back(
            {Eigen::Vector3f{point.x, point.y, point.z}, point.time});
        point_cloud.intensities.push_back(1.0f);
      }
    } else {
      pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
      pcl::fromROSMsg(msg, pcl_point_cloud);
      point_cloud.points.reserve(pcl_point_cloud.size());
      point_cloud.intensities.reserve(pcl_point_cloud.size());
      for (const auto& point : pcl_point_cloud) {
        point_cloud.points.push_back(
            {Eigen::Vector3f{point.x, point.y, point.z}, 0.f});
        point_cloud.intensities.push_back(1.0f);
      }
    }
  }
  ::cartographer::common::Time timestamp = FromRos(msg.header.stamp);
  if (!point_cloud.points.empty()) {
    const double duration = point_cloud.points.back().time;
    timestamp += cartographer::common::FromSeconds(duration);
    for (auto& point : point_cloud.points) {
      point.time -= duration;
      CHECK_LE(point.time, 0.f)
          << "Encountered a point with a larger stamp than "
             "the last point in the cloud.";
    }
  }
  return std::make_tuple(point_cloud, timestamp);
}

LandmarkData ToLandmarkData(const LandmarkList& landmark_list) {
  LandmarkData landmark_data;
  landmark_data.time = FromRos(landmark_list.header.stamp);
  for (const LandmarkEntry& entry : landmark_list.landmarks) {
    landmark_data.landmark_observations.push_back(
        {entry.id, ToRigid3d(entry.tracking_from_landmark_transform),
         entry.translation_weight, entry.rotation_weight});
  }
  return landmark_data;
}

Rigid3d ToRigid3d(const geometry_msgs::msg::TransformStamped& transform) {
  return Rigid3d(ToEigen(transform.transform.translation),
                 ToEigen(transform.transform.rotation));
}

Rigid3d ToRigid3d(const geometry_msgs::msg::Pose& pose) {
  return Rigid3d({pose.position.x, pose.position.y, pose.position.z},
                 ToEigen(pose.orientation));
}

Eigen::Vector3d ToEigen(const geometry_msgs::msg::Vector3& vector3) {
  return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
}

Eigen::Quaterniond ToEigen(const geometry_msgs::msg::Quaternion& quaternion) {
  return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y,
                            quaternion.z);
}

geometry_msgs::msg::Transform ToGeometryMsgTransform(const Rigid3d& rigid3d) {
  geometry_msgs::msg::Transform transform;
  transform.translation.x = rigid3d.translation().x();
  transform.translation.y = rigid3d.translation().y();
  transform.translation.z = rigid3d.translation().z();
  transform.rotation.w = rigid3d.rotation().w();
  transform.rotation.x = rigid3d.rotation().x();
  transform.rotation.y = rigid3d.rotation().y();
  transform.rotation.z = rigid3d.rotation().z();
  return transform;
}

geometry_msgs::msg::Pose ToGeometryMsgPose(const Rigid3d& rigid3d) {
  geometry_msgs::msg::Pose pose;
  pose.position = ToGeometryMsgPoint(rigid3d.translation());
  pose.orientation.w = rigid3d.rotation().w();
  pose.orientation.x = rigid3d.rotation().x();
  pose.orientation.y = rigid3d.rotation().y();
  pose.orientation.z = rigid3d.rotation().z();
  return pose;
}

geometry_msgs::msg::Point ToGeometryMsgPoint(const Eigen::Vector3d& vector3d) {
  geometry_msgs::msg::Point point;
  point.x = vector3d.x();
  point.y = vector3d.y();
  point.z = vector3d.z();
  return point;
}

Eigen::Vector3d LatLongAltToEcef(const double latitude, const double longitude,
                                 const double altitude) {
  // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates
  constexpr double a = 6378137.;  // semi-major axis, equator to center.
  constexpr double f = 1. / 298.257223563;
  constexpr double b = a * (1. - f);  // semi-minor axis, pole to center.
  constexpr double a_squared = a * a;
  constexpr double b_squared = b * b;
  constexpr double e_squared = (a_squared - b_squared) / a_squared;
  const double sin_phi = std::sin(cartographer::common::DegToRad(latitude));
  const double cos_phi = std::cos(cartographer::common::DegToRad(latitude));
  const double sin_lambda = std::sin(cartographer::common::DegToRad(longitude));
  const double cos_lambda = std::cos(cartographer::common::DegToRad(longitude));
  const double N = a / std::sqrt(1 - e_squared * sin_phi * sin_phi);
  const double x = (N + altitude) * cos_phi * cos_lambda;
  const double y = (N + altitude) * cos_phi * sin_lambda;
  const double z = (b_squared / a_squared * N + altitude) * sin_phi;

  return Eigen::Vector3d(x, y, z);
}

cartographer::transform::Rigid3d ComputeLocalFrameFromLatLong(
    const double latitude, const double longitude) {
  const Eigen::Vector3d translation = LatLongAltToEcef(latitude, longitude, 0.);
  const Eigen::Quaterniond rotation =
      Eigen::AngleAxisd(cartographer::common::DegToRad(latitude - 90.),
                        Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(cartographer::common::DegToRad(-longitude),
                        Eigen::Vector3d::UnitZ());
  return cartographer::transform::Rigid3d(rotation * -translation, rotation);
}

std::unique_ptr<nav_msgs::msg::OccupancyGrid> CreateOccupancyGridMsg(
    const cartographer::io::PaintSubmapSlicesResult& painted_slices,
    const double resolution, const std::string& frame_id,
    const rclcpp::Time& time) {
  auto occupancy_grid = absl::make_unique<nav_msgs::msg::OccupancyGrid>();

  const int width = cairo_image_surface_get_width(painted_slices.surface.get());
  const int height =
      cairo_image_surface_get_height(painted_slices.surface.get());

  occupancy_grid->header.stamp = time;
  occupancy_grid->header.frame_id = frame_id;
  occupancy_grid->info.map_load_time = time;
  occupancy_grid->info.resolution = resolution;
  occupancy_grid->info.width = width;
  occupancy_grid->info.height = height;
  occupancy_grid->info.origin.position.x =
      -painted_slices.origin.x() * resolution;
  occupancy_grid->info.origin.position.y =
      (-height + painted_slices.origin.y()) * resolution;
  occupancy_grid->info.origin.position.z = 0.;
  occupancy_grid->info.origin.orientation.w = 1.;
  occupancy_grid->info.origin.orientation.x = 0.;
  occupancy_grid->info.origin.orientation.y = 0.;
  occupancy_grid->info.origin.orientation.z = 0.;

  const uint32_t* pixel_data = reinterpret_cast<uint32_t*>(
      cairo_image_surface_get_data(painted_slices.surface.get()));
  occupancy_grid->data.reserve(width * height);
  for (int y = height - 1; y >= 0; --y) {
    for (int x = 0; x < width; ++x) {
      const uint32_t packed = pixel_data[y * width + x];
      const unsigned char color = packed >> 16;
      const unsigned char observed = packed >> 8;
      const int value =
          observed == 0
              ? -1
              : ::cartographer::common::RoundToInt((1. - color / 255.) * 100.);
      CHECK_LE(-1, value);
      CHECK_GE(100, value);
      occupancy_grid->data.push_back(value);
    }
  }

  return occupancy_grid;
}

}  // namespace cartographer_ros
