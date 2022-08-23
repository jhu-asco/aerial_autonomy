#include "aerial_autonomy/trackers/closest_id_tracking_strategy.h"

bool ClosestIdTrackingStrategy::initialize(
    const std::unordered_map<uint32_t, tf::Transform> &tracking_vectors) {
  std::tuple<uint32_t, tf::Transform> tracking_vector;
  if (!getClosest(tracking_vectors, tracking_vector)) {
    return false;
  }

  tracked_id_ = std::get<0>(tracking_vector);
  last_tracking_vector_ = tracking_vector;
  tracking_locked_ = true;
  tracking_retries_ = 0;
  return true;
}

bool ClosestIdTrackingStrategy::getTrackingVector(
    const std::unordered_map<uint32_t, tf::Transform> &tracking_vectors,
    std::tuple<uint32_t, tf::Transform> &tracking_vector) {
  if (!tracking_locked_) {
    return false;
  }

  auto find_id = tracking_vectors.find(tracked_id_);
  if (find_id != tracking_vectors.end()) {
    tracking_vector = std::make_tuple(tracked_id_, find_id->second);
    last_tracking_vector_ = tracking_vector;
    tracking_retries_ = 0;
    return true;
  } else if (tracking_retries_ < max_tracking_retries_) {
    tracking_vector = last_tracking_vector_;
    tracking_retries_++;
    return true;
  }

  // Lost track of the target. Strategy must be reinitialized
  tracking_locked_ = false;
  return false;
}

bool ClosestIdTrackingStrategy::getClosest(
    const std::unordered_map<uint32_t, tf::Transform> &tracking_vectors,
    std::tuple<uint32_t, tf::Transform> &tracking_vector) {
  if (tracking_vectors.empty()) {
    return false;
  }
  int closest_id = -1;
  double closest_norm = std::numeric_limits<double>::max();
  for (auto itr : tracking_vectors) {
    // Only consider the IDs that match the target base_id
    if (itr.first % id_factor_ == base_id_)
    {
      if (itr.second.getOrigin().length() < closest_norm) {
        closest_norm = itr.second.getOrigin().length();
        closest_id = itr.first;
      }
    }
  }
  // Check that a target was found
  if (closest_id < 0) {
    return false;
  }
  tracking_vector =
      std::make_tuple(closest_id, tracking_vectors.find(closest_id)->second);
  return true;
}