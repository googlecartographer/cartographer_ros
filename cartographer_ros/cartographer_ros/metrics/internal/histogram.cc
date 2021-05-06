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

#include "cartographer_ros/metrics/internal/histogram.h"

#include <algorithm>
#include <numeric>

#include "glog/logging.h"

namespace cartographer_ros {
namespace metrics {

using BucketBoundaries = ::cartographer::metrics::Histogram::BucketBoundaries;

Histogram::Histogram(const std::map<std::string, std::string>& labels,
                     const BucketBoundaries& bucket_boundaries)
    : labels_(labels),
      bucket_boundaries_(bucket_boundaries),
      bucket_counts_(bucket_boundaries.size() + 1) {
  absl::MutexLock lock(&mutex_);
  CHECK(std::is_sorted(std::begin(bucket_boundaries_),
                       std::end(bucket_boundaries_)));
}

void Histogram::Observe(double value) {
  auto bucket_index =
      std::distance(bucket_boundaries_.begin(),
                    std::upper_bound(bucket_boundaries_.begin(),
                                     bucket_boundaries_.end(), value));
  absl::MutexLock lock(&mutex_);
  sum_ += value;
  bucket_counts_[bucket_index] += 1;
}

std::map<double, double> Histogram::CountsByBucket() {
  absl::MutexLock lock(&mutex_);
  std::map<double, double> counts_by_bucket;
  // Add the finite buckets.
  for (size_t i = 0; i < bucket_boundaries_.size(); ++i) {
    counts_by_bucket[bucket_boundaries_.at(i)] = bucket_counts_.at(i);
  }
  // Add the "infinite" bucket.
  counts_by_bucket[kInfiniteBoundary] = bucket_counts_.back();
  return counts_by_bucket;
}

double Histogram::Sum() {
  absl::MutexLock lock(&mutex_);
  return sum_;
}

double Histogram::CumulativeCount() {
  absl::MutexLock lock(&mutex_);
  return std::accumulate(bucket_counts_.begin(), bucket_counts_.end(), 0.);
}

cartographer_ros_msgs::msg::Metric Histogram::ToRosMessage() {
  cartographer_ros_msgs::msg::Metric msg;
  msg.type = cartographer_ros_msgs::msg::Metric::TYPE_HISTOGRAM;
  for (const auto& label : labels_) {
    cartographer_ros_msgs::msg::MetricLabel label_msg;
    label_msg.key = label.first;
    label_msg.value = label.second;
    msg.labels.push_back(label_msg);
  }
  for (const auto& bucket : CountsByBucket()) {
    cartographer_ros_msgs::msg::HistogramBucket bucket_msg;
    bucket_msg.bucket_boundary = bucket.first;
    bucket_msg.count = bucket.second;
    msg.counts_by_bucket.push_back(bucket_msg);
  }
  return msg;
}

}  // namespace metrics
}  // namespace cartographer_ros
