#include "aerial_autonomy/trackers/simple_multi_tracker.h"

void SimpleMultiTracker::setTrackingVectors(
    const std::unordered_map<uint32_t, tf::Transform> &pose) {
  tracking_vectors_ = pose;
}

bool SimpleMultiTracker::trackingIsValid() { return true; }

bool SimpleMultiTracker::getTrackingVectors(
    std::unordered_map<uint32_t, tf::Transform> &pose) {
  pose = tracking_vectors_;
  return true;
}
