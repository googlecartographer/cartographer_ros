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

#include "cartographer_ros/metrics/family_factory.h"

#include "absl/memory/memory.h"

namespace cartographer_ros {
namespace metrics {

using BucketBoundaries = ::cartographer::metrics::Histogram::BucketBoundaries;

::cartographer::metrics::Family<::cartographer::metrics::Counter>*
FamilyFactory::NewCounterFamily(const std::string& name,
                                const std::string& description) {
  auto wrapper = absl::make_unique<CounterFamily>(name, description);
  auto* ptr = wrapper.get();
  counter_families_.emplace_back(std::move(wrapper));
  return ptr;
}

::cartographer::metrics::Family<::cartographer::metrics::Gauge>*
FamilyFactory::NewGaugeFamily(const std::string& name,
                              const std::string& description) {
  auto wrapper = absl::make_unique<GaugeFamily>(name, description);
  auto* ptr = wrapper.get();
  gauge_families_.emplace_back(std::move(wrapper));
  return ptr;
}

::cartographer::metrics::Family<::cartographer::metrics::Histogram>*
FamilyFactory::NewHistogramFamily(const std::string& name,
                                  const std::string& description,
                                  const BucketBoundaries& boundaries) {
  auto wrapper =
      absl::make_unique<HistogramFamily>(name, description, boundaries);
  auto* ptr = wrapper.get();
  histogram_families_.emplace_back(std::move(wrapper));
  return ptr;
}

void FamilyFactory::ReadMetrics(
    cartographer_ros_msgs::srv::ReadMetrics::Response::SharedPtr response) const {
  for (const auto& counter_family : counter_families_) {
    response->metric_families.push_back(counter_family->ToRosMessage());
  }
  for (const auto& gauge_family : gauge_families_) {
    response->metric_families.push_back(gauge_family->ToRosMessage());
  }
  for (const auto& histogram_family : histogram_families_) {
    response->metric_families.push_back(histogram_family->ToRosMessage());
  }
}

}  // namespace metrics
}  // namespace cartographer_ros
