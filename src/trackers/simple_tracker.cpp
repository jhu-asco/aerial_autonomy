#include "aerial_autonomy/trackers/simple_tracker.h"
#include "aerial_autonomy/trackers/simple_tracking_strategy.h"
#include <glog/logging.h>

SimpleTracker::SimpleTracker(parsernode::Parser &drone_hardware,
                             tf::Transform camera_transform)
    : BaseTracker(std::move(
          std::unique_ptr<TrackingStrategy>(new SimpleTrackingStrategy()))),
      drone_hardware_(drone_hardware), tracking_valid_(true),
      camera_transform_(camera_transform),
      update_tracker_pose_timer_(
          std::bind(&SimpleTracker::updateRelativePoses, this),
          std::chrono::milliseconds(10)) {
  update_tracker_pose_timer_.start();
}

void SimpleTracker::updateRelativePoses() {
  std::lock_guard<std::recursive_mutex> lock(timer_mutex_);
  if (!trackingIsValid()) {
    return;
  }
  std::unordered_map<uint32_t, tf::Transform> p;
  parsernode::common::quaddata uav_data;
  drone_hardware_.getquaddata(uav_data);
  tf::Transform quad_tf_global(
      tf::createQuaternionFromRPY(uav_data.rpydata.x, uav_data.rpydata.y,
                                  uav_data.rpydata.z),
      tf::Vector3(uav_data.localpos.x, uav_data.localpos.y,
                  uav_data.localpos.z));
  for (auto target : target_poses_) {
    p[target.first] =
        camera_transform_.inverse() * quad_tf_global.inverse() * target.second;
  }
  updateTrackingPoses(p);
}

void SimpleTracker::setTargetPosesGlobalFrame(
    std::unordered_map<uint32_t, tf::Transform> &poses) {
  std::lock_guard<std::recursive_mutex> lock(timer_mutex_);
  target_poses_ = poses;
  updateRelativePoses();
}

void SimpleTracker::setTargetPoseGlobalFrame(tf::Transform p) {
  std::lock_guard<std::recursive_mutex> lock(timer_mutex_);
  target_poses_[0] = p;
  updateRelativePoses();
}

void SimpleTracker::setTargetPositionGlobalFrame(Position p) {
  std::lock_guard<std::recursive_mutex> lock(timer_mutex_);
  target_poses_[0] =
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(p.x, p.y, p.z));
  updateRelativePoses();
}

bool SimpleTracker::trackingIsValid() { return tracking_valid_; }

void SimpleTracker::setTrackingIsValid(bool is_valid) {
  std::lock_guard<std::recursive_mutex> lock(timer_mutex_);
  tracking_valid_ = is_valid;
}

tf::Transform SimpleTracker::cameraTransform() { return camera_transform_; }
