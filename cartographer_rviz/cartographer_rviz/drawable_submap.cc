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

#include "cartographer_rviz/drawable_submap.h"

#include <chrono>
#include <future>
#include <sstream>
#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "eigen_conversions/eigen_msg.h"
#include "ros/ros.h"

namespace cartographer_rviz {

namespace {

constexpr std::chrono::milliseconds kMinQueryDelayInMs(250);

// Distance before which the submap will be shown at full opacity, and distance
// over which the submap will then fade out.
constexpr double kFadeOutStartDistanceInMeters = 1.;
constexpr double kFadeOutDistanceInMeters = 2.;
constexpr float kAlphaUpdateThreshold = 0.2f;

const Ogre::ColourValue kSubmapIdColor(Ogre::ColourValue::Red);
const Eigen::Vector3d kSubmapIdPosition(0.0, 0.0, 0.3);
constexpr float kSubmapIdCharHeight = 0.2f;

}  // namespace

DrawableSubmap::DrawableSubmap(const ::cartographer::mapping::SubmapId& id,
                               ::rviz::DisplayContext* const display_context,
                               Ogre::SceneNode* const map_node,
                               ::rviz::Property* const submap_category,
                               const bool visible, const float pose_axes_length,
                               const float pose_axes_radius)
    : id_(id),
      display_context_(display_context),
      submap_node_(map_node->createChildSceneNode()),
      submap_id_text_node_(submap_node_->createChildSceneNode()),
      ogre_submap_(id, display_context->getSceneManager(), submap_node_),
      pose_axes_(display_context->getSceneManager(), submap_node_,
                 pose_axes_length, pose_axes_radius),
      submap_id_text_(QString("(%1,%2)")
                          .arg(id.trajectory_id)
                          .arg(id.submap_index)
                          .toStdString()),
      last_query_timestamp_(0) {
  // DrawableSubmap creates and manages its visibility property object
  // (a unique_ptr is needed because the Qt parent of the visibility
  // property is the submap_category object - the BoolProperty needs
  // to be destroyed along with the DrawableSubmap)
  visibility_ = ::cartographer::common::make_unique<::rviz::BoolProperty>(
      "" /* title */, visible, "" /* description */, submap_category,
      SLOT(ToggleVisibility()), this);
  submap_id_text_.setCharacterHeight(kSubmapIdCharHeight);
  submap_id_text_.setColor(kSubmapIdColor);
  submap_id_text_.setTextAlignment(::rviz::MovableText::H_CENTER,
                                   ::rviz::MovableText::V_ABOVE);
  // TODO(jihoonl): Make it toggleable.
  submap_id_text_node_->setPosition(ToOgre(kSubmapIdPosition));
  submap_id_text_node_->attachObject(&submap_id_text_);
  submap_node_->setVisible(visible);
  connect(this, SIGNAL(RequestSucceeded()), this, SLOT(UpdateSceneNode()));
}

DrawableSubmap::~DrawableSubmap() {
  // 'query_in_progress_' must be true until the Q_EMIT has happened. Qt then
  // makes sure that 'RequestSucceeded' is not called after our destruction.
  if (QueryInProgress()) {
    rpc_request_future_.wait();
  }
  display_context_->getSceneManager()->destroySceneNode(submap_node_);
  display_context_->getSceneManager()->destroySceneNode(submap_id_text_node_);
}

void DrawableSubmap::Update(
    const ::std_msgs::Header& header,
    const ::cartographer_ros_msgs::SubmapEntry& metadata) {
  ::cartographer::common::MutexLocker locker(&mutex_);
  metadata_version_ = metadata.submap_version;
  pose_ = ::cartographer_ros::ToRigid3d(metadata.pose);
  submap_node_->setPosition(ToOgre(pose_.translation()));
  submap_node_->setOrientation(ToOgre(pose_.rotation()));
  display_context_->queueRender();
  visibility_->setName(
      QString("%1.%2").arg(id_.submap_index).arg(metadata_version_));
  visibility_->setDescription(
      QString("Toggle visibility of this individual submap.<br><br>"
              "Trajectory %1, submap %2, submap version %3")
          .arg(id_.trajectory_id)
          .arg(id_.submap_index)
          .arg(metadata_version_));
}

bool DrawableSubmap::MaybeFetchTexture(ros::ServiceClient* const client) {
  ::cartographer::common::MutexLocker locker(&mutex_);
  // Received metadata version can also be lower if we restarted Cartographer.
  const bool newer_version_available =
      submap_textures_ == nullptr ||
      submap_textures_->version != metadata_version_;
  const std::chrono::milliseconds now =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch());
  const bool recently_queried =
      last_query_timestamp_ + kMinQueryDelayInMs > now;
  if (!newer_version_available || recently_queried || query_in_progress_) {
    return false;
  }
  query_in_progress_ = true;
  last_query_timestamp_ = now;
  rpc_request_future_ = std::async(std::launch::async, [this, client]() {
    std::unique_ptr<::cartographer_ros::SubmapTextures> submap_textures =
        ::cartographer_ros::FetchSubmapTextures(id_, client);
    ::cartographer::common::MutexLocker locker(&mutex_);
    query_in_progress_ = false;
    if (submap_textures != nullptr && !submap_textures->textures.empty()) {
      // We emit a signal to update in the right thread, and pass via the
      // 'submap_texture_' member to simplify the signal-slot connection
      // slightly.
      submap_textures_ = std::move(submap_textures);
      Q_EMIT RequestSucceeded();
    }
  });
  return true;
}

bool DrawableSubmap::QueryInProgress() {
  ::cartographer::common::MutexLocker locker(&mutex_);
  return query_in_progress_;
}

void DrawableSubmap::SetAlpha(const double current_tracking_z) {
  const double distance_z =
      std::abs(pose_.translation().z() - current_tracking_z);
  const double fade_distance =
      std::max(distance_z - kFadeOutStartDistanceInMeters, 0.);
  const float target_alpha = static_cast<float>(
      std::max(0., 1. - fade_distance / kFadeOutDistanceInMeters));

  if (std::abs(target_alpha - current_alpha_) > kAlphaUpdateThreshold ||
      target_alpha == 0.f || target_alpha == 1.f) {
    current_alpha_ = target_alpha;
  }
  ogre_submap_.SetAlpha(current_alpha_);
  display_context_->queueRender();
}

void DrawableSubmap::UpdateSceneNode() {
  ::cartographer::common::MutexLocker locker(&mutex_);
  // TODO(gaschler): Add UI feature to show different textures.
  ogre_submap_.Update(submap_textures_->textures[0]);
  display_context_->queueRender();
}

void DrawableSubmap::ToggleVisibility() {
  submap_node_->setVisible(visibility_->getBool());
  display_context_->queueRender();
}

}  // namespace cartographer_rviz
