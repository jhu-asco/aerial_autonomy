#pragma once

#include "aerial_autonomy/trackers/tracking_strategy.h"

class ClosestTrackingStrategy : public TrackingStrategy {
public:
  ClosestTrackingStrategy() : tracking_locked_(false) {}
  virtual bool initialize(
      const std::vector<std::tuple<uint32_t, Position>> &tracking_vectors);
  virtual bool getTrackingVector(
      const std::vector<std::tuple<uint32_t, Position>> &tracking_vectors,
      std::tuple<uint32_t, Position> &tracking_vector);

private:
  /**
  * @brief Get the object with the closest position
  * @param tracking_vectors List of tracked targets
  * @param tracking_vector Returned closest target
  * @return True if success, false otherwise
  */
  bool getClosest(
      const std::vector<std::tuple<uint32_t, Position>> &tracking_vectors,
      std::tuple<uint32_t, Position> &tracking_vector);

  uint32_t tracked_id_;
  bool tracking_locked_;
};
