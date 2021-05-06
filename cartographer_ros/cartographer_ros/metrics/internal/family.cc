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

#include "cartographer_ros/metrics/internal/family.h"

#include "absl/memory/memory.h"
#include "cartographer_ros/metrics/internal/counter.h"
#include "cartographer_ros/metrics/internal/gauge.h"
#include "cartographer_ros/metrics/internal/histogram.h"

namespace cartographer_ros {
namespace metrics {

using BucketBoundaries = ::cartographer::metrics::Histogram::BucketBoundaries;

Counter* CounterFamily::Add(const std::map<std::string, std::string>& labels) {
  auto wrapper = absl::make_unique<Counter>(labels);
  auto* ptr = wrapper.get();
  wrappers_.emplace_back(std::move(wrapper));
  return ptr;
}

cartographer_ros_msgs::msg::MetricFamily CounterFamily::ToRosMessage() {
  cartographer_ros_msgs::msg::MetricFamily family_msg;
  family_msg.name = name_;
  family_msg.description = description_;
  for (const auto& wrapper : wrappers_) {
    family_msg.metrics.push_back(wrapper->ToRosMessage());
  }
  return family_msg;
}

Gauge* GaugeFamily::Add(const std::map<std::string, std::string>& labels) {
  auto wrapper = absl::make_unique<Gauge>(labels);
  auto* ptr = wrapper.get();
  wrappers_.emplace_back(std::move(wrapper));
  return ptr;
}

cartographer_ros_msgs::msg::MetricFamily GaugeFamily::ToRosMessage() {
  cartographer_ros_msgs::msg::MetricFamily family_msg;
  family_msg.name = name_;
  family_msg.description = description_;
  for (const auto& wrapper : wrappers_) {
    family_msg.metrics.push_back(wrapper->ToRosMessage());
  }
  return family_msg;
}

Histogram* HistogramFamily::Add(
    const std::map<std::string, std::string>& labels) {
  auto wrapper = absl::make_unique<Histogram>(labels, boundaries_);
  auto* ptr = wrapper.get();
  wrappers_.emplace_back(std::move(wrapper));
  return ptr;
}

cartographer_ros_msgs::msg::MetricFamily HistogramFamily::ToRosMessage() {
  cartographer_ros_msgs::msg::MetricFamily family_msg;
  family_msg.name = name_;
  family_msg.description = description_;
  for (const auto& wrapper : wrappers_) {
    family_msg.metrics.push_back(wrapper->ToRosMessage());
  }
  return family_msg;
}

}  // namespace metrics
}  // namespace cartographer_ros
