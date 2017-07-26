#pragma once
#include "aerial_autonomy/trackers/base_tracker.h"
#include "aerial_autonomy/trackers/closest_tracking_strategy.h"

#include <memory>
#include <tuple>
#include <vector>

/**
 * @brief Multi-object tracker
 */
class MultiTracker : public BaseTracker {
public:
  /**
  * @brief Defines the possible tracking strategies
  */
  enum TrackingStrategyType { CLOSEST };
  /**
  * @brief Default constructor
  */
  MultiTracker() : tracking_strategy_(new ClosestTrackingStrategy()) {}
  /**
  * @brief Constructor that takes a tracking strategy
  * @param tracking_strategy Strategy used to choose among multiple objects
  */
  MultiTracker(TrackingStrategyType tracking_strategy) {
    switch (tracking_strategy) {
    case (CLOSEST):
      tracking_strategy_.reset(new ClosestTrackingStrategy());
      break;
    }
  }

  /**
   * @brief Get the tracking vector
   * @param pos Returned tracking vector
   * @return True if successful, false otherwise
   */
  virtual bool getTrackingVector(Position &pos);
  /**
   * @brief Get the tracking vector
   * @param pos Returned tracking vector
   * @return True if successful, false otherwise
   */
  virtual bool getTrackingVector(std::tuple<uint32_t, Position> &pos);
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

  /**
  * @brief Initialize the tracking strategy
  */
  virtual bool initialize();

private:
  /**
  * @brief Strategy used to choose which object to track among multiple objects
  */
  std::unique_ptr<TrackingStrategy> tracking_strategy_;
};
