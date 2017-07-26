#include "aerial_autonomy/trackers/closest_tracking_strategy.h"

#include <limits>

bool ClosestTrackingStrategy::initialize(
    const std::vector<std::tuple<uint32_t, Position>> &tracking_vectors) {
  std::tuple<uint32_t, Position> tracking_vector;
  if (!getClosest(tracking_vectors, tracking_vector)) {
    return false;
  }

  tracked_id_ = std::get<0>(tracking_vector);
  tracking_locked_ = true;
  return true;
}

bool ClosestTrackingStrategy::getTrackingVector(
    const std::vector<std::tuple<uint32_t, Position>> &tracking_vectors,
    std::tuple<uint32_t, Position> &tracking_vector) {
  if (!tracking_locked_) {
    return false;
  }

  for (unsigned int i = 0; i < tracking_vectors.size(); i++) {
    if (std::get<0>(tracking_vectors[i]) == tracked_id_) {
      tracking_vector = tracking_vectors[i];
      return true;
    }
  }

  // If was lost track of the target, strategy must be reinitialized
  tracking_locked_ = false;
  return false;
}

bool ClosestTrackingStrategy::getClosest(
    const std::vector<std::tuple<uint32_t, Position>> &tracking_vectors,
    std::tuple<uint32_t, Position> &tracking_vector) {
  if (tracking_vectors.empty()) {
    return false;
  }
  int closest_idx = 0;
  double closest_norm = std::numeric_limits<double>::max();
  for (unsigned int i = 0; i < tracking_vectors.size(); i++) {
    if (std::get<1>(tracking_vectors[i]).norm() < closest_norm) {
      closest_norm = std::get<1>(tracking_vectors[i]).norm();
      closest_idx = i;
    }
  }
  tracking_vector = tracking_vectors[closest_idx];
  return true;
}
