#include "aerial_autonomy/trackers/multi_tracker.h"
#include <limits>

bool MultiTracker::getTrackingVector(Position &pos) {
  if (!trackingIsValid()) {
    return false;
  }
  std::vector<std::tuple<uint32_t, Position>> tracking_vectors;
  if (!getTrackingVectors(tracking_vectors) || tracking_vectors.empty()) {
    return false;
  }
  pos = getClosest(tracking_vectors);

  return true;
}

Position MultiTracker::getClosest(
    const std::vector<std::tuple<uint32_t, Position>> &positions) {
  int closest_idx = 0;
  double closest_norm = std::numeric_limits<double>::max();
  for (unsigned int i = 0; i < positions.size(); i++) {
    if (std::get<1>(positions[i]).norm() < closest_norm) {
      closest_norm = std::get<1>(positions[i]).norm();
      closest_idx = i;
    }
  }
  return std::get<1>(positions[closest_idx]);
}
