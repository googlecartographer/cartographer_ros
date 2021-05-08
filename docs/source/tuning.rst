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

.. cartographer SHA: aba4575d937df4c9697f61529200c084f2562584
.. cartographer_ros SHA: 99c23b6ac7874f7974e9ed808ace841da6f2c8b0
.. TODO(hrapp): mention insert_free_space somewhere

Tuning methodology
==================

Tuning Cartographer is unfortunately really difficult.
The system has many parameters many of which affect each other.
This tuning guide tries to explain a principled approach on concrete examples.

Built-in tools
--------------

Cartographer provides built-in tools for SLAM evaluation that can be particularly useful for measuring the local SLAM quality.
They are stand-alone executables that ship with the core ``cartographer`` library and are hence independent, but compatible with ``cartographer_ros``.
Therefore, please head to the `Cartographer Read the Docs Evaluation site`_ for a conceptual overview and a guide on how to use the tools in practice.

These tools assume that you have serialized the SLAM state to a ``.pbstream`` file.
With ``cartographer_ros``, you can invoke the ``assets_writer`` to serialize the state - see the :ref:`assets_writer` section for more information.

.. _Cartographer Read the Docs Evaluation site: https://google-cartographer.readthedocs.io/en/latest/evaluation.html

Example: tuning local SLAM
--------------------------

For this example we'll start at ``cartographer`` commit `aba4575`_ and ``cartographer_ros`` commit `99c23b6`_ and look at the bag ``b2-2016-04-27-12-31-41.bag`` from our test data set.

At our starting configuration, we see some slipping pretty early in the bag.
The backpack passed over a ramp in the Deutsches Museum which violates the 2D assumption of a flat floor.
It is visible in the laser scan data that contradicting information is passed to the SLAM.
But the slipping also indicates that we trust the point cloud matching too much and disregard the other sensors quite strongly.
Our aim is to improve the situation through tuning.

.. _aba4575: https://github.com/cartographer-project/cartographer/commit/aba4575d937df4c9697f61529200c084f2562584
.. _99c23b6: https://github.com/cartographer-project/cartographer_ros/commit/99c23b6ac7874f7974e9ed808ace841da6f2c8b0

If we only look at this particular submap, that the error is fully contained in one submap.
We also see that over time, global SLAM figures out that something weird happened and partially corrects for it.
The broken submap is broken forever though.

.. TODO(hrapp): VIDEO

Since the problem here is slippage inside a submap, it is a local SLAM issue.
So let's turn off global SLAM to not mess with our tuning.

.. code-block:: lua

   POSE_GRAPH.optimize_every_n_nodes = 0

Correct size of submaps
^^^^^^^^^^^^^^^^^^^^^^^

The size of submaps is configured through ``TRAJECTORY_BUILDER_2D.submaps.num_range_data``.
Looking at the individual submaps for this example they already fit the two constraints rather well, so we assume this parameter is well tuned.

Tuning the ``CeresScanMatcher``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In our case, the scan matcher can freely move the match forward and backwards without impacting the score.
We'd like to penalize this situation by making the scan matcher pay more for deviating from the prior that it got.
The two parameters controlling this are ``TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight`` and ``rotation_weight``.
The higher, the more expensive it is to move the result away from the prior, or in other words: scan matching has to generate a higher score in another position to be accepted.

For instructional purposes, let's make deviating from the prior really expensive:

.. code-block:: lua

   TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1e3

.. TODO(hrapp): video

This allows the optimizer to pretty liberally overwrite the scan matcher results.
This results in poses close to the prior, but inconsistent with the depth sensor and clearly broken.
Experimenting with this value yields a better result at ``2e2``.

.. TODO(hrapp): VIDEO with translation_weight = 2e2

Here, the scan matcher used rotation to still slightly mess up the result though.
Setting the ``rotation_weight`` to ``4e2`` leaves us with a reasonable result.

Verification
^^^^^^^^^^^^

To make sure that we did not overtune for this particular issue, we need to run the configuration against other collected data.
In this case, the new parameters did reveal slipping, for example at the beginning of ``b2-2016-04-05-14-44-52.bag``, so we had to lower the ``translation_weight`` to ``1e2``.
This setting is worse for the case we wanted to fix, but no longer slips.
Before checking them in, we normalize all weights, since they only have relative meaning.
The result of this tuning was `PR 428`_.
In general, always try to tune for a platform, not a particular bag.

.. _PR 428: https://github.com/cartographer-project/cartographer/pull/428

Special Cases
-------------

The default configuration and the above tuning steps are focused on quality.
Only after we have achieved good quality, we can further consider special cases.

Low Latency
^^^^^^^^^^^

By low latency, we mean that an optimized local pose becomes available shortly after sensor input was received,
usually within a second, and that global optimization has no backlog.
Low latency is required for online algorithms, such as robot localization.
Local SLAM, which operates in the foreground, directly affects latency.
Global SLAM builds up a queue of background tasks.
When global SLAM cannot keep up the queue, drift can accumulate indefinitely,
so global SLAM should be tuned to work in real time.

There are many options to tune the different components for speed, and we list them ordered from
the recommended, straightforward ones to the those that are more intrusive.
It is recommended to only explore one option at a time, starting with the first.
Configuration parameters are documented in the `Cartographer documentation`_.

.. _Cartographer documentation: https://google-cartographer.readthedocs.io/en/latest/configuration.html

To tune global SLAM for lower latency, we reduce its computational load
until is consistently keeps up with real-time input.
Below this threshold, we do not reduce it further, but try to achieve the best possible quality.
To reduce global SLAM latency, we can

- decrease ``optimize_every_n_nodes``
- increase ``MAP_BUILDER.num_background_threads`` up to the number of cores
- decrease ``global_sampling_ratio``
- decrease ``constraint_builder.sampling_ratio``
- increase ``constraint_builder.min_score``
- for the adaptive voxel filter(s), decrease ``.min_num_points``, ``.max_range``, increase ``.max_length``
- increase ``voxel_filter_size``, ``submaps.resolution``, decrease ``submaps.num_range_data``
- decrease search windows sizes, ``.linear_xy_search_window``, ``.linear_z_search_window``, ``.angular_search_window``
- increase ``global_constraint_search_after_n_seconds``
- decrease ``max_num_iterations``

To tune local SLAM for lower latency, we can

- increase ``voxel_filter_size``
- increase ``submaps.resolution``
- for the adaptive voxel filter(s), decrease ``.min_num_points``, ``.max_range``, increase ``.max_length``
- decrease ``max_range`` (especially if data is noisy)
- decrease ``submaps.num_range_data``

Note that larger voxels will slightly increase scan matching scores as a side effect,
so score thresholds should be increased accordingly.

Pure Localization in a Given Map
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Pure localization is different from mapping.
First, we expect a lower latency of both local and global SLAM.
Second, global SLAM will usually find a very large number of inter constraints between the frozen trajectory
that serves as a map and the current trajectory.

To tune for pure localization, we should first trim submaps and only keep those that are fresh.

..  code-block:: lua

   TRAJECTORY_BUILDER.pure_localization_trimmer = {
      max_submaps_to_keep = 3,
   }

Strongly decrease ``POSE_GRAPH.optimize_every_n_nodes`` to receive frequent results.
With these settings, global SLAM will usually be too slow and cannot keep up.
As a next step, we strongly decrease ``global_sampling_ratio`` and ``constraint_builder.sampling_ratio``
to compensate for the large number of constraints.
We then tune for lower latency as explained above until the system reliably works in real time.

If you run in localization-only mode, ``submaps.resolution`` **should be matching** with the resolution of the submaps in the ``.pbstream`` you are running on.
Using different resolutions is currently untested and may not work as expected.

Odometry in Global Optimization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If a separate odometry source is used as an input for local SLAM (``use_odometry = true``), we can also tune the global SLAM to benefit from this additional information.

There are in total four parameters that allow us to tune the individual weights of local SLAM and odometry in the optimization:

.. code-block:: lua

    POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight
    POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight
    POSE_GRAPH.optimization_problem.odometry_translation_weight
    POSE_GRAPH.optimization_problem.odometry_rotation_weight

We can set these weights depending on how much we trust either local SLAM or the odometry.
By default, odometry is weighted into global optimization similar to local slam (scan matching) poses.
However, odometry from wheel encoders often has a high uncertainty in rotation.
In this case, the rotation weight can be reduced, even down to zero.

Still have a problem ?
----------------------

If you can't get Cartographer to work reliably on your data, you can open a `GitHub issue`_ asking for help.
Developers are keen to help, but they can only be helpful if you follow `an issue template`_ containing the result of ``rosbag_validate``, a link to a fork of ``cartographer_ros`` with your config and a link to a ``.bag`` file reproducing your problem.

.. note::

   There are already lots of GitHub issues with all sorts of problems solved by the developers. Going through `the closed issues of cartographer_ros`_ and `of cartographer`_ is a great way to learn more about Cartographer and maybe find a solution to your problem !

.. _GitHub issue: https://github.com/cartographer-project/cartographer_ros/issues
.. _an issue template: https://github.com/cartographer-project/cartographer_ros/issues/new?labels=question
.. _the closed issues of cartographer_ros: https://github.com/cartographer-project/cartographer_ros/issues?q=is%3Aissue+is%3Aclosed
.. _of cartographer: https://github.com/cartographer-project/cartographer_ros/issues?q=is%3Aissue+is%3Aclosed
