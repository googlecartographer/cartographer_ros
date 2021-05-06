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

#ifndef CARTOGRAPHER_ROS_METRICS_INTERNAL_COUNTER_H
#define CARTOGRAPHER_ROS_METRICS_INTERNAL_COUNTER_H

#include "cartographer/metrics/counter.h"
#include "cartographer_ros/metrics/internal/gauge.h"
#include "cartographer_ros_msgs/msg/metric.hpp"

namespace cartographer_ros {
namespace metrics {

class Counter : public ::cartographer::metrics::Counter {
 public:
  explicit Counter(const std::map<std::string, std::string>& labels)
      : gauge_(labels) {}

  void Increment(const double value) override { gauge_.Increment(value); }

  void Increment() override { gauge_.Increment(); }

  double Value() { return gauge_.Value(); }

  cartographer_ros_msgs::msg::Metric ToRosMessage() {
    cartographer_ros_msgs::msg::Metric msg = gauge_.ToRosMessage();
    msg.type = cartographer_ros_msgs::msg::Metric::TYPE_COUNTER;
    return msg;
  }

 private:
  Gauge gauge_;
};

}  // namespace metrics
}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_METRICS_INTERNAL_COUNTER_H
