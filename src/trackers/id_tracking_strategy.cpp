#include "aerial_autonomy/trackers/id_tracking_strategy.h"

bool IdTrackingStrategy::initialize(
    const std::unordered_map<uint32_t, tf::Transform> &tracking_vectors) {
  return true;
}
bool IdTrackingStrategy::getTrackingVector(
    const std::unordered_map<uint32_t, tf::Transform> &tracking_vectors,
    std::tuple<uint32_t, tf::Transform> &tracking_vector) {
  auto itr = tracking_vectors.find(id_);
  bool success = false;
  if (itr != tracking_vectors.end()) {
    tracking_vector = std::make_tuple(itr->first, itr->second);
    success = true;
  }

  return success;
}
