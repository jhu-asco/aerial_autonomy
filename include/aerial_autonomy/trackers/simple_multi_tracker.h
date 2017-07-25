#pragma once
#include "aerial_autonomy/trackers/multi_tracker.h"

/**
 * @brief A simple MultiTracker implementation for testing
 */
class SimpleMultiTracker : public MultiTracker {
public:
  SimpleMultiTracker(TrackingStrategy tracking_strategy)
      : MultiTracker(tracking_strategy) {}
  /**
   * @brief Get the tracking vectors
   * @param pos Returned list of tracking vectors
   * @return True if successful, false otherwise
   */
  virtual bool
  getTrackingVectors(std::vector<std::tuple<uint32_t, Position>> &pos);
  /**
  * @brief Check whether tracking is valid
  * @return True if the tracking is valid, false otherwise
  */
  virtual bool trackingIsValid();

  /**
  * @brief Set the tracking vectors returned by the tracker
  * @param pos The tracking vectors to set
  */
  void
  setTrackingVectors(const std::vector<std::tuple<uint32_t, Position>> &pos);

private:
  /**
  * @brief Tracking vectors to return
  */
  std::vector<std::tuple<uint32_t, Position>> tracking_vectors_;
};
