.. Copyright 2018 The Cartographer Authors

.. Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

..      http://www.apache.org/licenses/LICENSE-2.0

.. Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

========================================
Running Cartographer ROS on your own bag
========================================

Now that you've run Cartographer ROS on a couple of provided bags, you can go ahead and make Cartographer work with your own data.
Find a ``.bag`` recording you would like to use for SLAM and go through this tutorial.

.. warning:: When you want to run cartographer_ros, you might need to source your ROS environment by running ``source install_isolated/setup.bash`` first (replace bash with zsh if your shell is zsh)

Validate your bag
=================

Cartographer ROS provides a tool named ``cartographer_rosbag_validate`` to automatically analyze data present in your bag.
It is generally a good idea to run this tool before trying to tune Cartographer for incorrect data.

It benefits from the experience of the Cartographer authors and can detect a variety of mistakes commonly found in bags.
For instance, if a ``sensor_msgs/Imu`` topic is detected, the tool will make sure that the gravity vector has not been removed from the IMU measurements because the gravity norm is used by Cartographer to determine the direction of the ground.

The tool can also provide tips on how to improve the quality of your data.
For example, with a Velodyne LIDAR, it is recommended to have one ``sensor_msgs/PointCloud2`` message per UDP packet sent by the sensor instead of one message per revolution.
With that granularity, Cartographer is then able to unwarp the point clouds deformation caused by the robot's motion and results in better reconstruction.

If you have sourced your Cartographer ROS environment, you can simply run the tool like this:

..  code-block:: bash

    cartographer_rosbag_validate -bag_filename your_bag.bag

Create a .lua configuration
===========================

Cartographer is highly flexible and can be configured to work on a variety of robots.
The robot configuration is read from a ``options`` data structure that must be defined from a Lua script.
The example configurations are defined in ``src/cartographer_ros/cartographer_ros/configuration_files`` and installed in ``install_isolated/share/cartographer_ros/configuration_files/``.

.. note:: Ideally, a .lua configuration should be robot-specific and not bag-specific.

You can start by copying one of the example and then adapt it to your own need. If you want to use 3D SLAM:

..  code-block:: bash

    cp install_isolated/share/cartographer_ros/configuration_files/backpack_3d.lua install_isolated/share/cartographer_ros/configuration_files/my_robot.lua
 
If you want to use 2D SLAM:

..  code-block:: bash

    cp install_isolated/share/cartographer_ros/configuration_files/backpack_2d.lua install_isolated/share/cartographer_ros/configuration_files/my_robot.lua

You can then edit ``my_robot.lua`` to suit the needs of your robot.
The values defined in the ``options`` block define how the Cartographer ROS frontend should interface with your bag.
The values defined after the ``options`` paragraph are used to tune the inner-working of Cartographer, we will ignore these for now.

.. seealso:: The `reference documentation of the Cartographer ROS configuration values`_ and of `the Cartographer configuration values`_.

.. _reference documentation of the Cartographer ROS configuration values: https://google-cartographer-ros.readthedocs.io/en/latest/configuration.html

.. _the Cartographer configuration values: https://google-cartographer.readthedocs.io/en/latest/configuration.html

Among the values you need to adapt, you probably have to provide the TF frame IDs of your environment and robot in ``map_frame``, ``tracking_frame``, ``published_frame`` and ``odom_frame``.

.. note:: You can either distribute your robot's TF tree from a ``/tf`` topic in your bag or define it in a ``.urdf`` robot definition.

.. warning:: You should trust your poses! A small offset on the link between your robot and IMU or LIDAR can lead to incoherent map reconstructions. Cartographer can usually correct small pose errors but not everything!

The other values you need to define are related to the number and type of sensors you would like to use.

- ``num_laser_scans``: Number of ``sensor_msgs/LaserScan`` topics you'll use.
- ``num_multi_echo_laser_scans``: Number of ``sensor_msgs/MultiEchoLaserScan`` topics you'll use.
- ``num_point_clouds``: Number of ``sensor_msgs/PointCloud2`` topics you'll use.

You can also enable the usage of landmarks and GPS as additional sources of localization using ``use_landmarks`` and ``use_nav_sat``. The rest of the variables in the ``options`` block should typically be left untouched.

.. note:: even if you use a 2D SLAM, the landmarks are 3D objects and can mislead you if viewed only on the 2D plane due to their third dimension.

However, there is one global variable that you absolutely need to adapt to the needs of your bag: ``TRAJECTORY_BUILDER_3D.num_accumulated_range_data`` or ``TRAJECTORY_BUILDER_2D.num_accumulated_range_data``.
This variable defines the number of messages required to construct a full scan (typically, a full revolution).
If you follow ``cartographer_rosbag_validate``'s advices and use 100 ROS messages per scan, you can set this variable to 100.
If you have two range finding sensors (for instance, two LIDARs) providing their full scans all at once, you should set this variable to 2.

Create .launch files for your SLAM scenarios
============================================

You may have noticed that each demo introduced in the previous section was run with a different roslaunch command.
The recommended usage of Cartographer is indeed to provide a custom ``.launch`` file per robot and type of SLAM.
The example ``.launch`` files are defined in ``src/cartographer_ros/cartographer_ros/launch`` and installed in ``install_isolated/share/cartographer_ros/launch/``.

Start by copying one of the provided example:

..  code-block:: bash

    cp install_isolated/share/cartographer_ros/launch/backpack_3d.launch install_isolated/share/cartographer_ros/launch/my_robot.launch
    cp install_isolated/share/cartographer_ros/launch/demo_backpack_3d.launch install_isolated/share/cartographer_ros/launch/demo_my_robot.launch
    cp install_isolated/share/cartographer_ros/launch/offline_backpack_3d.launch install_isolated/share/cartographer_ros/launch/offline_my_robot.launch
    cp install_isolated/share/cartographer_ros/launch/demo_backpack_3d_localization.launch install_isolated/share/cartographer_ros/launch/demo_my_robot_localization.launch
    cp install_isolated/share/cartographer_ros/launch/assets_writer_backpack_3d.launch install_isolated/share/cartographer_ros/launch/assets_writer_my_robot.launch

- ``my_robot.launch`` is meant to be used on the robot to execute SLAM online (in real time) with real sensors data.
- ``demo_my_robot.launch`` is meant to be used from a development machine and expects a ``bag_filename`` argument to replay data from a recording. This launch file also spawns a rviz window configured to visualize Cartographer's state.
- ``offline_my_robot.launch`` is very similar to ``demo_my_robot.launch`` but tries to execute SLAM as fast as possible. This can make map building significantly faster. This launch file can also use multiple bag files provided to the ``bag_filenames`` argument.
- ``demo_my_robot_localization.launch`` is very similar to ``demo_my_robot.launch`` but expects a ``load_state_filename`` argument pointing to a ``.pbstream`` recording of a previous Cartographer execution. The previous recording will be used as a pre-computed map and Cartographer will only perform localization on this map.
- ``assets_writer_my_robot.launch`` is used to extract data out of a ``.pbstream`` recording of a previous Cartographer execution.

Again, a few adaptations need to be made to those files to suit your robot.

- Every parameter given to ``-configuration_basename`` should be adapted to point to ``my_robot.lua``.
- If you decided to use a ``.urdf`` description of your robot, you should place your description in ``install_isolated/share/cartographer_ros/urdf`` and adapt the ``robot_description`` parameter to point to your file name.
- If you decided to use ``/tf`` messages, you can remove the ``robot_description`` parameter, the ``robot_state_publisher`` node and the lines statring with ``-urdf``.
- If the topic names published by your bag or sensors don't match the ones expected by Cartographer ROS, you can use ``<remap>`` elements to redirect your topics. The expected topic names depend on the type of range finding devices you use.

.. note::

    - The IMU topic is expected to be named "imu"
    - If you use only one ``sensor_msgs/LaserScan`` topic, it is expected to be named ``scan``. If you have more, they should be named ``scan_1``, ``scan_2`` etc...
    - If you use only one ``sensor_msgs/MultiEchoLaserScan`` topic, it is expected to be named ``echoes``. If you have more, they should be named ``echoes_1``, ``echoes_2`` etc...
    - If you use only one ``sensor_msgs/PointCloud2`` topic, it is expected be named ``points2``. If you have more, they should be named ``points2_1``, ``points2_2``, etc...

Try your configuration
======================

Everything is setup! You can now start Cartographer with:

..  code-block:: bash

    roslaunch cartographer_ros my_robot.launch bag_filename:=/path/to/your_bag.bag

If you are lucky enough, everything should already work as expected.
However, you might have some problems that require tuning.
