#pragma once
#include "aerial_autonomy/types/position.h"

/**
 * @brief Interface for classes that provide a vector
 * to a tracked target
 */
class BaseTracker {
public:
  /**
  * @brief Initialze the tracker.  Can simply return true if the subclass
  * requires no additional initialization.
  * @return True if initialization succeeds, false otherwise
  */
  virtual bool initialize() { return true; }
  /**
   * @brief Get the tracking vector
   * @param pos Returned tracking vector
   * @return True if successful, false otherwise
   */
  virtual bool getTrackingVector(Position &pos) = 0;
  /**
  * @brief Check whether tracking is valid
  * @return True if the tracking is valid, false otherwise
  */
  virtual bool trackingIsValid() = 0;
};
