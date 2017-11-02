#pragma once

#include "aerial_autonomy/trackers/tracking_strategy.h"

/**
 * @brief A tracking strategy that locks on to a specific ID.
 */
class IdTrackingStrategy : public TrackingStrategy {
public:
  /**
  * @brief Constructor
  * @param id ID to track
  */
  IdTrackingStrategy(uint32_t id) : id_(id) {}
  /**
  * @brief Initialize the strategy. Does nothing.
  * @param tracking_vectors The vectors to the tracked targets
  * @return True if successful, false otherwise
  */
  virtual bool initialize(
      const std::unordered_map<uint32_t, tf::Transform> &tracking_vectors);
  /**
  * @brief Get the tracking vector for the tracked target.
  * @param tracking_vectors The vectors to the tracked targets
  * @param tracking_vector Tracked target with specified ID
  * @return True if successful, false otherwise
  */
  virtual bool getTrackingVector(
      const std::unordered_map<uint32_t, tf::Transform> &tracking_vectors,
      std::tuple<uint32_t, tf::Transform> &tracking_vector);

private:
  /**
  * @brief ID of tracked target
  */
  uint32_t id_;
};
