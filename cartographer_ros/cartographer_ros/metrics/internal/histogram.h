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

#ifndef CARTOGRAPHER_ROS_METRICS_INTERNAL_HISTOGRAM_H
#define CARTOGRAPHER_ROS_METRICS_INTERNAL_HISTOGRAM_H

#include <map>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "cartographer/metrics/histogram.h"
#include "cartographer_ros_msgs/msg/metric.hpp"

namespace cartographer_ros {
namespace metrics {

constexpr double kInfiniteBoundary = std::numeric_limits<double>::infinity();

using BucketBoundaries = ::cartographer::metrics::Histogram::BucketBoundaries;

class Histogram : public ::cartographer::metrics::Histogram {
 public:
  explicit Histogram(const std::map<std::string, std::string>& labels,
                     const BucketBoundaries& bucket_boundaries);

  void Observe(double value) override;

  std::map<double, double> CountsByBucket();

  double Sum();

  double CumulativeCount();

  cartographer_ros_msgs::msg::Metric ToRosMessage();

 private:
  absl::Mutex mutex_;
  const std::map<std::string, std::string> labels_;
  const BucketBoundaries bucket_boundaries_;
  std::vector<double> bucket_counts_ GUARDED_BY(mutex_);
  double sum_ GUARDED_BY(mutex_);
};

}  // namespace metrics
}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_METRICS_INTERNAL_HISTOGRAM_H
