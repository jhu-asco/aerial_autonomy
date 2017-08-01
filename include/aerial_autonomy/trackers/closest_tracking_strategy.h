#pragma once

#include "aerial_autonomy/trackers/tracking_strategy.h"

/**
 * @brief A tracking strategy that locks on to the closest target when
 * initialized
 */
class ClosestTrackingStrategy : public TrackingStrategy {
public:
  /**
  * @brief Constructor
  */
  ClosestTrackingStrategy() : tracking_locked_(false) {}
  /**
  * @brief Initialize the strategy from a group of targets. Locks on to closest
  * target
  * @param tracking_vectors The vectors to the tracked targets
  * @return True if successful, false otherwise
  */
  virtual bool
  initialize(const std::unordered_map<uint32_t, Position> &tracking_vectors);
  /**
  * @brief Get the tracking vector for the tracked target.
  * @param tracking_vectors The vectors to the tracked targets
  * @param tracking_vector Chosen target
  * @return True if successful, false otherwise
  */
  virtual bool getTrackingVector(
      const std::unordered_map<uint32_t, Position> &tracking_vectors,
      std::tuple<uint32_t, Position> &tracking_vector);

private:
  /**
  * @brief Get the object with the closest position
  * @param tracking_vectors List of tracked targets
  * @param tracking_vector Returned closest target
  * @return True if success, false otherwise
  */
  bool
  getClosest(const std::unordered_map<uint32_t, Position> &tracking_vectors,
             std::tuple<uint32_t, Position> &tracking_vector);

  /**
  * @brief ID of tracked target
  */
  uint32_t tracked_id_;
  /**
  * @brief True if tracking has been locked, false otherwise
  */
  bool tracking_locked_;
};
