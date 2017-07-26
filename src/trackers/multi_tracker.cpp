#include "aerial_autonomy/trackers/multi_tracker.h"

bool MultiTracker::getTrackingVector(Position &pos) {
  std::tuple<uint32_t, Position> pos_tuple;
  if (!getTrackingVector(pos_tuple)) {
    return false;
  }
  pos = std::get<1>(pos_tuple);
  return true;
}

bool MultiTracker::getTrackingVector(std::tuple<uint32_t, Position> &pos) {
  if (!trackingIsValid()) {
    return false;
  }
  std::vector<std::tuple<uint32_t, Position>> tracking_vectors;
  if (!getTrackingVectors(tracking_vectors) || tracking_vectors.empty()) {
    return false;
  }

  if (!tracking_strategy_->getTrackingVector(tracking_vectors, pos)) {
    return false;
  }

  return true;
}

bool MultiTracker::initialize() {
  std::vector<std::tuple<uint32_t, Position>> tracking_vectors;
  if (!getTrackingVectors(tracking_vectors)) {
    return false;
  }
  return tracking_strategy_->initialize(tracking_vectors);
}
