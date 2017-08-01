#include "aerial_autonomy/trackers/simple_multi_tracker.h"

void SimpleMultiTracker::setTrackingVectors(
    const std::unordered_map<uint32_t, Position> &pos) {
  tracking_vectors_ = pos;
}

bool SimpleMultiTracker::trackingIsValid() { return true; }

bool SimpleMultiTracker::getTrackingVectors(
    std::unordered_map<uint32_t, Position> &pos) {
  pos = tracking_vectors_;
  return true;
}
