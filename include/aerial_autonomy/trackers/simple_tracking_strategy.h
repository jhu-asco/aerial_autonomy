#pragma once

#include "aerial_autonomy/trackers/tracking_strategy.h"

/**
 * @brief A strategy that simply returns the first element in the list of
 * tracking vectors
 * Should be used if you expect your tracker to only return a single element
 */
class SimpleTrackingStrategy : public TrackingStrategy {
public:
  /**
  * @brief Does nothing here since we are always initialized
  * @param tracking_vectors The vectors to the tracked targets
  * @return Always returns true
  */
  virtual bool initialize(
      const std::unordered_map<uint32_t, tf::Transform> &tracking_vectors);
  /**
  * @brief Get the first tracking vector
  * @param tracking_vectors The vectors to the tracked targets
  * @param tracking_vector First tracking vector
  * @return True if successful, false otherwise
  */
  virtual bool getTrackingVector(
      const std::unordered_map<uint32_t, tf::Transform> &tracking_vectors,
      std::tuple<uint32_t, tf::Transform> &tracking_vector);
};
