#pragma once
#include "aerial_autonomy/trackers/base_tracker.h"
#include <parsernode/parser.h>
#include <tf/tf.h>

/**
 * @brief Simple tracker for a given fixed position.
 */
class SimpleTracker : public BaseTracker {
public:
  SimpleTracker(parsernode::Parser &drone_hardware);
  /**
   * @brief Get the tracking vector
   * @param pos Returned tracking vector
   * @return True if successful, false otherwise
   */
  virtual bool getTrackingVector(Position &pos);
  /**
  * @brief Check whether tracking is valid
  * @return True if the tracking is valid, false otherwise
  */
  virtual bool trackingIsValid();

  /**
   * @brief Set whether the tracking is valid
   * @param is_valid True if the tracking should be valid false otherwise
   */
  void setTrackingIsValid(bool is_valid);

  /**
   * @brief Set the target position in the global frame
   * @param p Position to set
   */
  void setTargetPositionGlobalFrame(Position p);

  /**
   * @brief Get a reference to the camera transform
   * @return Reference to the camera transform
   */
  tf::Transform &cameraTransform();

private:
  parsernode::Parser &drone_hardware_;
  bool tracking_valid_;
  Position target_position_;
  /**
   * @brief Transform of camera in uav frame
   */
  tf::Transform
      camera_transform_; /// \todo Matt set this to something somewhere
};
