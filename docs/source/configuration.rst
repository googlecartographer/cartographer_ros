.. Copyright 2016 The Cartographer Authors

.. Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

..      http://www.apache.org/licenses/LICENSE-2.0

.. Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

=========================================
Lua configuration reference documentation
=========================================

Note that Cartographer's ROS integration uses `tf2`_, thus all frame IDs are
expected to contain only a frame name (lower-case with underscores) and no
prefix or slashes. See `REP 105`_ for commonly used coordinate frames.

Note that topic names are given as *base* names (see `ROS Names`_) in
Cartographer's ROS integration. This means it is up to the user of the
Cartographer node to remap, or put them into a namespace.

The following are Cartographer's ROS integration top-level options, all of which
must be specified in the Lua configuration file:

map_frame
  The ROS frame ID to use for publishing submaps, the parent frame of poses,
  usually "map".

tracking_frame
  The ROS frame ID of the frame that is tracked by the SLAM algorithm. If an IMU
  is used, it should be at its position, although it might be rotated. A common
  choice is "imu_link".

published_frame
  The ROS frame ID to use as the child frame for publishing poses. For example
  "odom" if an "odom" frame is supplied by a different part of the system. In
  this case the pose of "odom" in the *map_frame* will be published. Otherwise,
  setting it to "base_link" is likely appropriate.

odom_frame
  Only used if *provide_odom_frame* is true. The frame between *published_frame*
  and *map_frame* to be used for publishing the (non-loop-closed) local SLAM
  result. Usually "odom".

provide_odom_frame
  If enabled, the local, non-loop-closed, continuous pose will be published as
  the *odom_frame* in the *map_frame*.

publish_frame_projected_to_2d
  If enabled, the published pose will be restricted to a pure 2D pose (no roll,
  pitch, or z-offset). This prevents potentially unwanted out-of-plane poses in
  2D mode that can occur due to the pose extrapolation step (e.g. if the pose
  shall be published as a 'base-footprint'-like frame)

use_odometry
  If enabled, subscribes to `nav_msgs/Odometry`_ on the topic "odom". Odometry
  must be provided in this case, and the information will be included in SLAM.

use_nav_sat
  If enabled, subscribes to `sensor_msgs/NavSatFix`_ on the topic "fix".
  Navigation data must be provided in this case, and the information will be
  included in the global SLAM.

use_landmarks
  If enabled, subscribes to `cartographer_ros_msgs/LandmarkList`_ on the topic
  "landmarks".  Landmarks must be provided, as `cartographer_ros_msgs/LandmarkEntry`_ within `cartographer_ros_msgs/LandmarkList`_.  If `cartographer_ros_msgs/LandmarkEntry`_ data is provided the information
  will be included in the SLAM accoding to the ID of the `cartographer_ros_msgs/LandmarkEntry`_. The `cartographer_ros_msgs/LandmarkList`_ should be provided at a sample rate comparable to the other sensors.  The list can be empty but has to be provided because Cartographer strictly time orders sensor data in order to make the landmarks deterministic. However it is possible to set the trajectory builder option "collate_landmarks" to false and allow for a non-deterministic but also non-blocking approach.

num_laser_scans
  Number of laser scan topics to subscribe to. Subscribes to
  `sensor_msgs/LaserScan`_ on the "scan" topic for one laser scanner, or topics
  "scan_1", "scan_2", etc. for multiple laser scanners.

num_multi_echo_laser_scans
  Number of multi-echo laser scan topics to subscribe to. Subscribes to
  `sensor_msgs/MultiEchoLaserScan`_ on the "echoes" topic for one laser scanner,
  or topics "echoes_1", "echoes_2", etc. for multiple laser scanners.

num_subdivisions_per_laser_scan
  Number of point clouds to split each received (multi-echo) laser scan into.
  Subdividing a scan makes it possible to unwarp scans acquired while the
  scanners are moving. There is a corresponding trajectory builder option to
  accumulate the subdivided scans into a point cloud that will be used for scan
  matching.

num_point_clouds
  Number of point cloud topics to subscribe to. Subscribes to
  `sensor_msgs/PointCloud2`_ on the "points2" topic for one rangefinder, or
  topics "points2_1", "points2_2", etc. for multiple rangefinders.

lookup_transform_timeout_sec
  Timeout in seconds to use for looking up transforms using `tf2`_.

submap_publish_period_sec
  Interval in seconds at which to publish the submap poses, e.g. 0.3 seconds.

pose_publish_period_sec
  Interval in seconds at which to publish poses, e.g. 5e-3 for a frequency of
  200 Hz.

publish_to_tf
  Enable or disable providing of TF transforms.

publish_tracked_pose
  Enable publishing of tracked pose as a `geometry_msgs/PoseStamped`_ to topic "tracked_pose".

trajectory_publish_period_sec
  Interval in seconds at which to publish the trajectory markers, e.g. 30e-3
  for 30 milliseconds.

rangefinder_sampling_ratio
  Fixed ratio sampling for range finders messages.

odometry_sampling_ratio
  Fixed ratio sampling for odometry messages.

fixed_frame_sampling_ratio
  Fixed ratio sampling for fixed frame messages.

imu_sampling_ratio
  Fixed ratio sampling for IMU messages.

landmarks_sampling_ratio
  Fixed ratio sampling for landmarks messages.

.. _REP 105: http://www.ros.org/reps/rep-0105.html
.. _ROS Names: http://wiki.ros.org/Names
.. _geometry_msgs/PoseStamped: http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html
.. _nav_msgs/OccupancyGrid: http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
.. _nav_msgs/Odometry: http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html
.. _sensor_msgs/LaserScan: http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
.. _sensor_msgs/MultiEchoLaserScan: http://docs.ros.org/api/sensor_msgs/html/msg/MultiEchoLaserScan.html
.. _sensor_msgs/PointCloud2: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
.. _sensor_msgs/NavSatFix: http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html
.. _cartographer_ros_msgs/LandmarkList: https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros_msgs/msg/LandmarkList.msg
.. _cartographer_ros_msgs/LandmarkEntry: https://github.com/cartographer-project/cartographer_ros/blob/4b39ee68c7a4d518bf8d01a509331e2bc1f514a0/cartographer_ros_msgs/msg/LandmarkEntry.msg
.. _tf2: http://wiki.ros.org/tf2
