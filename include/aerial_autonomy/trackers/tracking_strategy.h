#pragma once

#include "aerial_autonomy/types/position.h"

#include <tuple>
#include <unordered_map>

/**
 * @brief Defines a strategy for choosing a target to track among a group of
 * tracked targets
 */
class TrackingStrategy {
public:
  /**
  * @brief Initialize the strategy from a group of targets
  * @param tracking_vectors The vectors to the tracked targets
  * @return True if successful, false otherwise
  */
  virtual bool initialize(
      const std::unordered_map<uint32_t, Position> &tracking_vectors) = 0;
  /**
  * @brief Get the tracking vector for one target based on the implemented
  * strategy
  * @param tracking_vectors The vectors to the tracked targets
  * @param tracking_vector Chosen target
  * @return True if successful, false otherwise
  */
  virtual bool getTrackingVector(
      const std::unordered_map<uint32_t, Position> &tracking_vectors,
      std::tuple<uint32_t, Position> &tracking_vector) = 0;
};
