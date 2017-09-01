#include "aerial_autonomy/trackers/simple_multi_tracker.h"

void SimpleMultiTracker::setTrackingVectors(
    const std::unordered_map<uint32_t, tf::Transform> &pos) {
  tracking_vectors_ = pos;
}

bool SimpleMultiTracker::trackingIsValid() { return true; }

bool SimpleMultiTracker::getTrackingVectors(
    std::unordered_map<uint32_t, tf::Transform> &pos) {
  pos = tracking_vectors_;
  return true;
}
