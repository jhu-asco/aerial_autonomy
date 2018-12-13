#include "aerial_autonomy/trackers/simple_multi_tracker.h"

void SimpleMultiTracker::setTrackingVectors(
    const std::unordered_map<uint32_t, tf::Transform> &pose) {
  updateTrackingPoses(pose);
}

bool SimpleMultiTracker::trackingIsValid() { return true; }
