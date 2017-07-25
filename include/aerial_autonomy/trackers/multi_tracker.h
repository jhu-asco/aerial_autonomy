#pragma once
#include "aerial_autonomy/trackers/base_tracker.h"

#include <tuple>
#include <vector>

/**
 * @brief Multi-object tracker
 */
class MultiTracker : public BaseTracker {
public:
  enum TrackingStrategy { CLOSEST };
  /**
  * @brief Default constructor
  */
  MultiTracker() : tracking_strategy_(CLOSEST) {}
  /**
  * @brief Constructor that takes a tracking strategy
  * @param tracking_strategy Strategy used to choose among multiple objects
  */
  MultiTracker(TrackingStrategy tracking_strategy)
      : tracking_strategy_(tracking_strategy) {}
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
  /**
  * @brief Get the object with the closest position
  * @param positions List of tracked objects
  * @return Closest object tracked
  */
  Position
  getClosest(const std::vector<std::tuple<uint32_t, Position>> &positions);

  /**
  * @brief Strategy used to choose which object to track among multiple objects
  */
  TrackingStrategy tracking_strategy_;
};
