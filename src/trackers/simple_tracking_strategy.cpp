#include "aerial_autonomy/trackers/simple_tracking_strategy.h"

bool SimpleTrackingStrategy::initialize(
    const std::unordered_map<uint32_t, Position> &tracking_vectors) {
  return true;
}
bool SimpleTrackingStrategy::getTrackingVector(
    const std::unordered_map<uint32_t, Position> &tracking_vectors,
    std::tuple<uint32_t, Position> &tracking_vector) {
  if (tracking_vectors.empty()) {
    return false;
  }

  auto first_vector = tracking_vectors.begin();
  tracking_vector = std::make_tuple(first_vector->first, first_vector->second);

  return true;
}
