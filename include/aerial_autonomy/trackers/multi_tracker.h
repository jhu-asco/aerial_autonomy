#pragma once
#include "aerial_autonomy/trackers/base_tracker.h"

#include <tuple>
#include <vector>

/**
 * @brief Multi-object tracker
 */
class MultiTracker : public BaseTracker {
public:
  /**
   * @brief Get the tracking vector
   * @param pos Returned tracking vector
   * @return True if successful, false otherwise
   */
  virtual bool getTrackingVector(Position &pos);
  /**
   * @brief Get the tracking vectors
   * @param pos Returned list of tracking vectors
   * @return True if successful, false otherwise
   */
  virtual bool
  getTrackingVectors(std::vector<std::tuple<uint32_t, Position>> &pos) = 0;
  /**
  * @brief Check whether tracking is valid
  * @return True if the tracking is valid, false otherwise
  */
  virtual bool trackingIsValid() = 0;

private:
  Position
  getClosest(const std::vector<std::tuple<uint32_t, Position>> &positions);
};
