#pragma once
#include "uav_vision_system_config.pb.h"
#include <aerial_autonomy/common/async_timer.h>
#include <aerial_autonomy/trackers/base_tracker.h>
#include <aerial_autonomy/types/position.h>
#include <mutex>
#include <parsernode/parser.h>
#include <tf/tf.h>

/**
 * @brief Simple tracker for a set of given fixed positions.
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
   * @brief Set the target pose in the global frame
   * @param p tf::Transform to set
   */
  void setTargetPoseGlobalFrame(tf::Transform p);

  /**
   * @brief Set the target poses in the global frame
   * @param poses Map of IDs to poses to set
   */
  void
  setTargetPosesGlobalFrame(std::unordered_map<uint32_t, tf::Transform> &poses);

  /**
   * @brief Set the target postion in the global frame.  Sets rotation to
   * identity.
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
  std::unordered_map<uint32_t, tf::Transform> target_poses_; ///< Tracked poses
  tf::Transform camera_transform_; ///< Transform of camera in uav frame
  AsyncTimer
      update_tracker_pose_timer_; ///< Update tracker poses based on quad pose
  std::recursive_mutex timer_mutex_; ///< Lock access to target poses
  void updateRelativePoses();
};
