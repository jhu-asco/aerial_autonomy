#pragma once
#include "aerial_autonomy/common/atomic.h"
#include "aerial_autonomy/trackers/tracking_strategy.h"

#include <tf/tf.h>

#include <chrono>
#include <tuple>
#include <unordered_map>

/**
 * @brief Interface for classes that provide a vector
 * to a tracked target
 */
class BaseTracker {
public:
  /**
  * @brief Constructor
  * @param tracking_strategy  Strategy used to pick a particular target from a
  * list of tracked objects
  */
  BaseTracker(std::unique_ptr<TrackingStrategy> &&tracking_strategy)
      : tracking_strategy_(std::move(tracking_strategy)) {}
  /**
  * @brief Initialze the tracker.  Can simply return true if the subclass
  * requires no additional initialization.
  * @return True if initialization succeeds, false otherwise
  */
  virtual bool initialize();
  /**
  * @brief Set the tracker's tracking strategy
  * @param The desired tracking strategy
  */
  void
  setTrackingStrategy(std::unique_ptr<TrackingStrategy> &&tracking_strategy);
  /**
   * @brief Get the tracking vector
   * @param pos Returned tracking vector
   * @return True if successful, false otherwise
   */
  bool getTrackingVector(tf::Transform &pos);
  /**
   * @brief Get the tracking vector
   * @param pos Returned tracking vector
   * @return True if successful, false otherwise
   */
  bool getTrackingVector(std::tuple<uint32_t, tf::Transform> &pos);
  /**
   * @brief Get the tracking vectors
   * @param pos Returned map of tracking vectors
   * @return True if successful, false otherwise
   */
  bool getTrackingVectors(std::unordered_map<uint32_t, tf::Transform> &pos);
  /**
  * @brief Check whether tracking is valid
  * @return True if the tracking is valid, false otherwise
  */
  virtual bool trackingIsValid() = 0;

  /**
  * @brief Get the time stamp of the current tracking vectors
  */
  // \todo Matt Remove this function and add time stamps to information stored
  // with tracking vector
  virtual std::chrono::time_point<std::chrono::high_resolution_clock>
  getTrackingTime() {
    return std::chrono::high_resolution_clock::now();
  }

  /**
   * @brief Set a callback function that will be called whenever a tracker
   * matching tracking strategy is available
   *
   * @param tracker_callback Callback function
   */
  void setTrackerCallback(
      std::function<void(uint32_t, tf::Transform)> tracker_callback);

protected:
  /**
   * @brief Update internal tracking vectors. Subclasses should call this
   * function to
   * update the tracking vectors
   *
   * @param tracking_poses Map of tracking transforms and their ids
   */
  void updateTrackingPoses(
      const std::unordered_map<uint32_t, tf::Transform> &tracking_poses);

private:
  /**
  * @brief Strategy used to choose which object to track among multiple objects
  */
  std::unique_ptr<TrackingStrategy> tracking_strategy_;
  /**
   * @brief Callback function when tracked object is available
   */
  std::function<void(uint32_t, tf::Transform)> tracker_callback_;
  /**
   * @brief Map of tracker ids and poses
   */
  Atomic<std::unordered_map<uint32_t, tf::Transform>> tracking_poses_;
};
