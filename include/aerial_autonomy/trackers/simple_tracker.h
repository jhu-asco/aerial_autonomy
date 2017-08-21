#pragma once
#include "uav_vision_system_config.pb.h"
#include <aerial_autonomy/trackers/base_tracker.h>
#include <parsernode/parser.h>
#include <tf/tf.h>

/**
 * @brief Simple tracker for a given fixed position.
 */
class SimpleTracker : public BaseTracker {
public:
  /**
  * @brief Constructor that stores drone hardware
  * and camera transform for further use
  *
  * @param drone_hardware UAV Hardware for getting sensor data
  * @param camera_transform Camera transform from UAV base
  */
  SimpleTracker(parsernode::Parser &drone_hardware,
                tf::Transform camera_transform);
  /**
   * @brief Get the tracking vector
   * @param pos Returned tracking vector
   * @return True if successful, false otherwise
   */
  virtual bool getTrackingVectors(std::unordered_map<uint32_t, Position> &pos);
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
   * @brief Get the camera transform stored
   * @return Reference to the camera transform
   */
  tf::Transform cameraTransform();

private:
  parsernode::Parser &drone_hardware_; ///< UAV Hardware
  bool tracking_valid_;                ///< Flag to specify if tracking is valid
  Position target_position_;           ///< Goal position to track
  tf::Transform camera_transform_;     ///< Transform of camera in uav frame
};
