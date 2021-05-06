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

#ifndef CARTOGRAPHER_ROS_METRICS_INTERNAL_FAMILY_H
#define CARTOGRAPHER_ROS_METRICS_INTERNAL_FAMILY_H

#include <memory>
#include <string>

#include "cartographer/metrics/family_factory.h"
#include "cartographer_ros/metrics/internal/counter.h"
#include "cartographer_ros/metrics/internal/gauge.h"
#include "cartographer_ros/metrics/internal/histogram.h"
#include "cartographer_ros_msgs/msg/metric_family.hpp"

namespace cartographer_ros {
namespace metrics {
class CounterFamily
    : public ::cartographer::metrics::Family<::cartographer::metrics::Counter> {
 public:
  CounterFamily(const std::string& name, const std::string& description)
      : name_(name), description_(description) {}
  Counter* Add(const std::map<std::string, std::string>& labels) override;
  cartographer_ros_msgs::msg::MetricFamily ToRosMessage();

 private:
  std::string name_;
  std::string description_;
  std::vector<std::unique_ptr<Counter>> wrappers_;
};

class GaugeFamily
    : public ::cartographer::metrics::Family<::cartographer::metrics::Gauge> {
 public:
  GaugeFamily(const std::string& name, const std::string& description)
      : name_(name), description_(description) {}
  Gauge* Add(const std::map<std::string, std::string>& labels) override;

  cartographer_ros_msgs::msg::MetricFamily ToRosMessage();

 private:
  std::string name_;
  std::string description_;
  std::vector<std::unique_ptr<Gauge>> wrappers_;
};

class HistogramFamily : public ::cartographer::metrics::Family<
                            ::cartographer::metrics::Histogram> {
 public:
  HistogramFamily(const std::string& name, const std::string& description,
                  const BucketBoundaries& boundaries)
      : name_(name), description_(description), boundaries_(boundaries) {}

  Histogram* Add(const std::map<std::string, std::string>& labels) override;

  cartographer_ros_msgs::msg::MetricFamily ToRosMessage();

 private:
  std::string name_;
  std::string description_;
  std::vector<std::unique_ptr<Histogram>> wrappers_;
  const BucketBoundaries boundaries_;
};

}  // namespace metrics
}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_METRICS_INTERNAL_FAMILY_H
