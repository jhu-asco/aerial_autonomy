#pragma once
#include "aerial_autonomy/trackers/base_tracker.h"

/**
 * @brief A simple Multi-object Tracker implementation for testing
 */
class SimpleMultiTracker : public BaseTracker {
public:
  /**
  * @brief Constructor
  * @param tracking_strategy Strategy to use for picking target among a group of
  * tracked objects
  */
  SimpleMultiTracker(std::unique_ptr<TrackingStrategy> tracking_strategy)
      : BaseTracker(std::move(tracking_strategy)) {}
  /**
   * @brief Get the tracking vectors
   * @param pos Returned list of tracking vectors
   * @return True if successful, false otherwise
   */
  virtual bool
  getTrackingVectors(std::unordered_map<uint32_t, tf::Transform> &pos);
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
  setTrackingVectors(const std::unordered_map<uint32_t, tf::Transform> &pos);

private:
  /**
  * @brief Tracking vectors to return
  */
  std::unordered_map<uint32_t, tf::Transform> tracking_vectors_;
};
