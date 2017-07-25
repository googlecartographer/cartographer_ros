/*
 * Copyright 2016 The Cartographer Authors
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

#ifndef CARTOGRAPHER_RVIZ_SRC_DRAWABLE_TRAJECTORY_H_
#define CARTOGRAPHER_RVIZ_SRC_DRAWABLE_TRAJECTORY_H_

#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/properties/bool_property.h"

#include "cartographer/mapping/id.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_rviz/drawable_submap.h"
#include "std_msgs/Header.h"

namespace cartographer_rviz {
class PerTrajectorySubmapDisplay : public QObject {
  Q_OBJECT

 public:
  using SubmapId = ::cartographer::mapping::SubmapId;

  PerTrajectorySubmapDisplay(int trajectory_id, ::rviz::Property* submap_category,
                     ::rviz::DisplayContext* display_context, bool visible);
  ~PerTrajectorySubmapDisplay() override;
  PerTrajectorySubmapDisplay(const PerTrajectorySubmapDisplay&) = delete;
  PerTrajectorySubmapDisplay& operator=(const PerTrajectorySubmapDisplay&) = delete;

  bool visibility() const { return visible_->getBool(); }
  void set_visibility(const bool visibility) { visible_->setBool(visibility); }

  bool IsTrajectoryInvaild(
      const ::cartographer_ros_msgs::SubmapEntry& submap_entry);

  // Updates the 'metadata' for this submap. If necessary, the next call to
  // MaybeFetchTexture() will fetch a new submap texture.
  void ProcessMessage(
      const ::std_msgs::Header& header,
      const ::cartographer_ros_msgs::SubmapEntry& submap_entry);

  void RemoveUnlistedSubmaps(const std::set<SubmapId> listed_submap);

  void SetAlpha(float current_tracking_z);
  void FetchTexture(ros::ServiceClient* const client);

 private Q_SLOTS:
  void AllEnabledToggled();

 private:
  const int id_;
  ::rviz::DisplayContext* const display_context_;

  std::map<int, std::unique_ptr<DrawableSubmap>> submaps_;

  std::unique_ptr<::rviz::BoolProperty> visible_;
};

}  // namespace cartographer_rviz

#endif  // CARTOGRAPHER_RVIZ_SRC_DRAWABLE_TRAJECTORY_H_
