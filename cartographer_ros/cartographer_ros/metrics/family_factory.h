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

#ifndef CARTOGRAPHER_ROS_METRICS_FAMILY_FACTORY_H
#define CARTOGRAPHER_ROS_METRICS_FAMILY_FACTORY_H

#include <memory>
#include <string>

#include "cartographer/metrics/family_factory.h"
#include "cartographer_ros/metrics/internal/counter.h"
#include "cartographer_ros/metrics/internal/family.h"
#include "cartographer_ros/metrics/internal/gauge.h"
#include "cartographer_ros/metrics/internal/histogram.h"
#include "cartographer_ros_msgs/srv/read_metrics.hpp"

namespace cartographer_ros {
namespace metrics {

// Realizes the factory / registry interface for the metrics in libcartographer
// and provides a wrapper to collect ROS messages from the metrics it owns.
class FamilyFactory : public ::cartographer::metrics::FamilyFactory {
 public:
  ::cartographer::metrics::Family<::cartographer::metrics::Counter>*

  NewCounterFamily(const std::string& name,
                   const std::string& description) override;
  ::cartographer::metrics::Family<::cartographer::metrics::Gauge>*
  NewGaugeFamily(const std::string& name,
                 const std::string& description) override;
  ::cartographer::metrics::Family<::cartographer::metrics::Histogram>*
  NewHistogramFamily(const std::string& name, const std::string& description,
                     const ::cartographer::metrics::Histogram::BucketBoundaries&
                         boundaries) override;

  void ReadMetrics(
      cartographer_ros_msgs::srv::ReadMetrics::Response::SharedPtr response) const;

 private:
  std::vector<std::unique_ptr<CounterFamily>> counter_families_;
  std::vector<std::unique_ptr<GaugeFamily>> gauge_families_;
  std::vector<std::unique_ptr<HistogramFamily>> histogram_families_;
};

}  // namespace metrics
}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_METRICS_FAMILY_FACTORY_H
