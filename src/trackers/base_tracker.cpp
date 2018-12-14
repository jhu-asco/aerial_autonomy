#include "aerial_autonomy/trackers/base_tracker.h"

bool BaseTracker::getTrackingVector(tf::Transform &pose) {
  std::tuple<uint32_t, tf::Transform> pose_tuple;
  if (!getTrackingVector(pose_tuple)) {
    return false;
  }
  pose = std::get<1>(pose_tuple);
  return true;
}

void BaseTracker::setTrackingStrategy(
    std::unique_ptr<TrackingStrategy> &&tracking_strategy) {
  tracking_strategy_ = std::move(tracking_strategy);
}

bool BaseTracker::getTrackingVector(std::tuple<uint32_t, tf::Transform> &pose) {
  if (!trackingIsValid()) {
    return false;
  }
  std::unordered_map<uint32_t, tf::Transform> tracking_vectors;
  if (!getTrackingVectors(tracking_vectors)) {
    return false;
  }

  if (!tracking_strategy_->getTrackingVector(tracking_vectors, pose)) {
    return false;
  }

  return true;
}

bool BaseTracker::initialize() {
  std::unordered_map<uint32_t, tf::Transform> tracking_vectors;
  if (!getTrackingVectors(tracking_vectors)) {
    return false;
  }
  return tracking_strategy_->initialize(tracking_vectors);
}

bool BaseTracker::getTrackingVectors(
    std::unordered_map<uint32_t, tf::Transform> &pos) {
  if (!trackingIsValid()) {
    return false;
  }
  pos = tracking_poses_;
  return true;
}

void BaseTracker::setTrackerCallback(
    std::function<void(uint32_t, tf::Transform)> tracker_callback) {
  tracker_callback_ = tracker_callback;
}

void BaseTracker::updateTrackingPoses(
    const std::unordered_map<uint32_t, tf::Transform> &tracking_poses) {
  std::tuple<uint32_t, tf::Transform> tracking_pose_tup;
  if (tracker_callback_ != nullptr &&
      tracking_strategy_->getTrackingVector(tracking_poses,
                                            tracking_pose_tup)) {
    tracker_callback_(std::get<0>(tracking_pose_tup),
                      std::get<1>(tracking_pose_tup));
  }
  tracking_poses_ = tracking_poses;
}
