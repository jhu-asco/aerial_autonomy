#pragma once

#include "aerial_autonomy/trackers/tracking_strategy.h"

/**
 * @brief A tracking strategy that locks on to the closest target when
 * initialized.
 */
class ClosestTrackingStrategy : public TrackingStrategy {
public:
  /**
  * @brief Constructor
  * @param max_tracking_retries Number of times to retry tracking before lock is
  * lost
  */
  ClosestTrackingStrategy(uint32_t max_tracking_retries = 20)
      : tracking_locked_(false), max_tracking_retries_(max_tracking_retries),
        tracking_retries_(0) {}
  /**
  * @brief Initialize the strategy from a group of targets. Locks on to closest
  * target
  * @param tracking_vectors The vectors to the tracked targets
  * @return True if successful, false otherwise
  */
  virtual bool initialize(
      const std::unordered_map<uint32_t, tf::Transform> &tracking_vectors);
  /**
  * @brief Get the tracking vector for the tracked target.
  * @param tracking_vectors The vectors to the tracked targets
  * @param tracking_vector Tracked target which was closest when initialized
  * @return True if successful, false otherwise
  */
  virtual bool getTrackingVector(
      const std::unordered_map<uint32_t, tf::Transform> &tracking_vectors,
      std::tuple<uint32_t, tf::Transform> &tracking_vector);

private:
  /**
  * @brief Get the object with the closest position
  * @param tracking_vectors List of tracked targets
  * @param tracking_vector Returned closest target
  * @return True if success, false otherwise
  */
  bool getClosest(
      const std::unordered_map<uint32_t, tf::Transform> &tracking_vectors,
      std::tuple<uint32_t, tf::Transform> &tracking_vector);

  /**
  * @brief ID of tracked target
  */
  uint32_t tracked_id_;
  /**
  * @brief True if tracking has been locked, false otherwise
  */
  bool tracking_locked_;
  /**
  * @brief Number of times to retry tracking before lock is lost
  */
  const uint32_t max_tracking_retries_;
  /**
  * @brief Number of tracking retries since last success
  */
  uint32_t tracking_retries_;
  /**
  * @brief Last successful tracking vector
  */
  std::tuple<uint32_t, tf::Transform> last_tracking_vector_;
};
