#include "aerial_autonomy/trackers/base_tracker.h"

bool BaseTracker::getTrackingVector(tf::Transform &pos) {
  std::tuple<uint32_t, tf::Transform> pos_tuple;
  if (!getTrackingVector(pos_tuple)) {
    return false;
  }
  pos = std::get<1>(pos_tuple);
  return true;
}

bool BaseTracker::getTrackingVector(std::tuple<uint32_t, tf::Transform> &pos) {
  if (!trackingIsValid()) {
    return false;
  }
  std::unordered_map<uint32_t, tf::Transform> tracking_vectors;
  if (!getTrackingVectors(tracking_vectors)) {
    return false;
  }

  if (!tracking_strategy_->getTrackingVector(tracking_vectors, pos)) {
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
