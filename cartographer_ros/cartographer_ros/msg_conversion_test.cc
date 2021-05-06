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
#include <random>

#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "cartographer_ros/time_conversion.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "sensor_msgs/LaserScan.h"

namespace cartographer_ros {
namespace {

using ::cartographer::sensor::LandmarkData;
using ::cartographer::sensor::LandmarkObservation;
using ::testing::AllOf;
using ::testing::DoubleNear;
using ::testing::ElementsAre;
using ::testing::Field;

constexpr double kEps = 1e-6;

TEST(MsgConversion, LaserScanToPointCloud) {
  sensor_msgs::msg::LaserScan laser_scan;
  for (int i = 0; i < 8; ++i) {
    laser_scan.ranges.push_back(1.f);
  }
  laser_scan.angle_min = 0.f;
  laser_scan.angle_max = 8.f * static_cast<float>(M_PI_4);
  laser_scan.angle_increment = static_cast<float>(M_PI_4);
  laser_scan.range_min = 0.f;
  laser_scan.range_max = 10.f;

  const auto point_cloud =
      std::get<0>(ToPointCloudWithIntensities(laser_scan)).points;
  EXPECT_TRUE(
      point_cloud[0].position.isApprox(Eigen::Vector3f(1.f, 0.f, 0.f), kEps));
  EXPECT_TRUE(point_cloud[1].position.isApprox(
      Eigen::Vector3f(1.f / std::sqrt(2.f), 1.f / std::sqrt(2.f), 0.f), kEps));
  EXPECT_TRUE(
      point_cloud[2].position.isApprox(Eigen::Vector3f(0.f, 1.f, 0.f), kEps));
  EXPECT_TRUE(point_cloud[3].position.isApprox(
      Eigen::Vector3f(-1.f / std::sqrt(2.f), 1.f / std::sqrt(2.f), 0.f), kEps));
  EXPECT_TRUE(
      point_cloud[4].position.isApprox(Eigen::Vector3f(-1.f, 0.f, 0.f), kEps));
  EXPECT_TRUE(point_cloud[5].position.isApprox(
      Eigen::Vector3f(-1.f / std::sqrt(2.f), -1.f / std::sqrt(2.f), 0.f),
      kEps));
  EXPECT_TRUE(
      point_cloud[6].position.isApprox(Eigen::Vector3f(0.f, -1.f, 0.f), kEps));
  EXPECT_TRUE(point_cloud[7].position.isApprox(
      Eigen::Vector3f(1.f / std::sqrt(2.f), -1.f / std::sqrt(2.f), 0.f), kEps));
  for (int i = 0; i < 8; ++i) {
    EXPECT_NEAR(point_cloud[i].time, 0.f, kEps);
  }
}

TEST(MsgConversion, LaserScanToPointCloudWithInfinityAndNaN) {
  sensor_msgs::msg::LaserScan laser_scan;
  laser_scan.ranges.push_back(1.f);
  laser_scan.ranges.push_back(std::numeric_limits<float>::infinity());
  laser_scan.ranges.push_back(2.f);
  laser_scan.ranges.push_back(std::numeric_limits<float>::quiet_NaN());
  laser_scan.ranges.push_back(3.f);
  laser_scan.angle_min = 0.f;
  laser_scan.angle_max = 3.f * static_cast<float>(M_PI_4);
  laser_scan.angle_increment = static_cast<float>(M_PI_4);
  laser_scan.range_min = 2.f;
  laser_scan.range_max = 10.f;

  const auto point_cloud =
      std::get<0>(ToPointCloudWithIntensities(laser_scan)).points;
  ASSERT_EQ(2, point_cloud.size());
  EXPECT_TRUE(
      point_cloud[0].position.isApprox(Eigen::Vector3f(0.f, 2.f, 0.f), kEps));
  EXPECT_TRUE(
      point_cloud[1].position.isApprox(Eigen::Vector3f(-3.f, 0.f, 0.f), kEps));
  EXPECT_NEAR(point_cloud[0].time, 0.f, kEps);
  EXPECT_NEAR(point_cloud[1].time, 0.f, kEps);
}

::testing::Matcher<const LandmarkObservation&> EqualsLandmark(
    const LandmarkObservation& expected) {
  return ::testing::AllOf(
      Field(&LandmarkObservation::id, expected.id),
      Field(&LandmarkObservation::landmark_to_tracking_transform,
            ::cartographer::transform::IsNearly(
                expected.landmark_to_tracking_transform, kEps)),
      Field(&LandmarkObservation::translation_weight,
            DoubleNear(expected.translation_weight, kEps)),
      Field(&LandmarkObservation::rotation_weight,
            DoubleNear(expected.rotation_weight, kEps)));
}

TEST(MsgConversion, LandmarkListToLandmarkData) {
  cartographer_ros_msgs::msg::LandmarkList message;
  message.header.stamp.fromSec(10);

  cartographer_ros_msgs::msg::LandmarkEntry landmark_0;
  landmark_0.id = "landmark_0";
  landmark_0.tracking_from_landmark_transform.position.x = 1.0;
  landmark_0.tracking_from_landmark_transform.position.y = 2.0;
  landmark_0.tracking_from_landmark_transform.position.z = 3.0;
  landmark_0.tracking_from_landmark_transform.orientation.w = 1.0;
  landmark_0.tracking_from_landmark_transform.orientation.x = 0.0;
  landmark_0.tracking_from_landmark_transform.orientation.y = 0.0;
  landmark_0.tracking_from_landmark_transform.orientation.z = 0.0;
  landmark_0.translation_weight = 1.0;
  landmark_0.rotation_weight = 2.0;
  message.landmarks.push_back(landmark_0);

  LandmarkData actual_landmark_data = ToLandmarkData(message);
  EXPECT_THAT(actual_landmark_data,
              AllOf(Field(&LandmarkData::time, FromRos(message.header.stamp)),
                    Field(&LandmarkData::landmark_observations,
                          ElementsAre(EqualsLandmark(LandmarkObservation{
                              "landmark_0",
                              ::cartographer::transform::Rigid3d(
                                  Eigen::Vector3d(1., 2., 3.),
                                  Eigen::Quaterniond(1., 0., 0., 0.)),
                              1.f,
                              2.f,
                          })))));
}

TEST(MsgConversion, LatLongAltToEcef) {
  Eigen::Vector3d equator_prime_meridian = LatLongAltToEcef(0, 0, 0);
  EXPECT_TRUE(equator_prime_meridian.isApprox(Eigen::Vector3d(6378137, 0, 0)))
      << equator_prime_meridian;
  Eigen::Vector3d plus_ten_meters = LatLongAltToEcef(0, 0, 10);
  EXPECT_TRUE(plus_ten_meters.isApprox(Eigen::Vector3d(6378147, 0, 0)))
      << plus_ten_meters;
  Eigen::Vector3d north_pole = LatLongAltToEcef(90, 0, 0);
  EXPECT_TRUE(north_pole.isApprox(Eigen::Vector3d(0, 0, 6356752.3142), kEps))
      << north_pole;
  Eigen::Vector3d also_north_pole = LatLongAltToEcef(90, 90, 0);
  EXPECT_TRUE(
      also_north_pole.isApprox(Eigen::Vector3d(0, 0, 6356752.3142), kEps))
      << also_north_pole;
  Eigen::Vector3d south_pole = LatLongAltToEcef(-90, 0, 0);
  EXPECT_TRUE(south_pole.isApprox(Eigen::Vector3d(0, 0, -6356752.3142), kEps))
      << south_pole;
  Eigen::Vector3d above_south_pole = LatLongAltToEcef(-90, 60, 20);
  EXPECT_TRUE(
      above_south_pole.isApprox(Eigen::Vector3d(0, 0, -6356772.3142), kEps))
      << above_south_pole;
  Eigen::Vector3d somewhere_on_earth =
      LatLongAltToEcef(48.1372149, 11.5748024, 517.1);
  EXPECT_TRUE(somewhere_on_earth.isApprox(
      Eigen::Vector3d(4177983, 855702, 4727457), kEps))
      << somewhere_on_earth;
}

TEST(MsgConversion, ComputeLocalFrameFromLatLong) {
  cartographer::transform::Rigid3d north_pole =
      ComputeLocalFrameFromLatLong(90., 0.);
  EXPECT_TRUE((north_pole * LatLongAltToEcef(90., 0., 1.))
                  .isApprox(Eigen::Vector3d::UnitZ()));
  cartographer::transform::Rigid3d south_pole =
      ComputeLocalFrameFromLatLong(-90., 0.);
  EXPECT_TRUE((south_pole * LatLongAltToEcef(-90., 0., 1.))
                  .isApprox(Eigen::Vector3d::UnitZ()));
  cartographer::transform::Rigid3d prime_meridian_equator =
      ComputeLocalFrameFromLatLong(0., 0.);
  EXPECT_TRUE((prime_meridian_equator * LatLongAltToEcef(0., 0., 1.))
                  .isApprox(Eigen::Vector3d::UnitZ()))
      << prime_meridian_equator * LatLongAltToEcef(0., 0., 1.);
  cartographer::transform::Rigid3d meridian_90_equator =
      ComputeLocalFrameFromLatLong(0., 90.);
  EXPECT_TRUE((meridian_90_equator * LatLongAltToEcef(0., 90., 1.))
                  .isApprox(Eigen::Vector3d::UnitZ()))
      << meridian_90_equator * LatLongAltToEcef(0., 90., 1.);

  std::mt19937 rng(42);
  std::uniform_real_distribution<double> lat_distribution(-90., 90.);
  std::uniform_real_distribution<double> long_distribution(-180., 180.);
  std::uniform_real_distribution<double> alt_distribution(-519., 519.);

  for (int i = 0; i < 1000; ++i) {
    const double latitude = lat_distribution(rng);
    const double longitude = long_distribution(rng);
    const double altitude = alt_distribution(rng);
    cartographer::transform::Rigid3d transform_to_local_frame =
        ComputeLocalFrameFromLatLong(latitude, longitude);
    EXPECT_TRUE((transform_to_local_frame *
                 LatLongAltToEcef(latitude, longitude, altitude))
                    .isApprox(altitude * Eigen::Vector3d::UnitZ(), kEps))
        << transform_to_local_frame *
               LatLongAltToEcef(latitude, longitude, altitude)
        << "\n"
        << altitude;
  }
}

}  // namespace
}  // namespace cartographer_ros
