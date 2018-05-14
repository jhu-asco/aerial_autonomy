#pragma once
#include "aerial_autonomy/controller_connectors/base_controller_connector.h"
#include "aerial_autonomy/controllers/relative_pose_controller.h"
#include "aerial_autonomy/trackers/base_tracker.h"

#include <arm_parsers/arm_parser.h>

#include <parsernode/parser.h>

#include <tf/tf.h>

/**
 * @brief A visual servoing controller that uses a tracker output as feedback
 * and moves the arm to a goal pose relative to the tracked target
 */
class VisualServoingControllerArmConnector
    : public ControllerConnector<std::tuple<tf::Transform, tf::Transform>,
                                 tf::Transform, tf::Transform> {
public:
  /**
   * @brief Constructor
   */
  VisualServoingControllerArmConnector(BaseTracker &tracker,
                                       parsernode::Parser &drone_hardware,
                                       ArmParser &arm_hardware,
                                       RelativePoseController &controller,
                                       tf::Transform camera_transform,
                                       tf::Transform arm_transform)
      : ControllerConnector(controller, ControllerGroup::Arm),
        drone_hardware_(drone_hardware), arm_hardware_(arm_hardware),
        tracker_(tracker), camera_transform_(camera_transform),
        arm_transform_(arm_transform) {}
  /**
   * @brief Destructor
   */
  virtual ~VisualServoingControllerArmConnector() {}

  /**
   * @brief Get the tracking pose of the tracker in the arm
   * frame
   * @param tracking_vector Returned tracking pose
   * @return True if successful and false otherwise
   */
  bool getTrackingPoseArmFrame(tf::Transform &tracking_pose);

protected:
  /**
   * @brief Extracts pose data from ROI
   *
   * @param sensor_data Pose of arm end effector and pose of tracked object
   * in arm frame
   *
   * @return true if able to extract ROI position
   */
  virtual bool
  extractSensorData(std::tuple<tf::Transform, tf::Transform> &sensor_data);

  /**
   * @brief  Send position commands to hardware
   *
   * @param controls position command to send to arm
   */
  virtual void sendControllerCommands(tf::Transform controls);

private:
  /**
  * @brief Drone hardware to send commands
  */
  parsernode::Parser &drone_hardware_;
  /**
  * @brief Arm hardware to send commands
  */
  ArmParser &arm_hardware_;
  /**
  * @brief Tracks whatever we are servoing to
  */
  BaseTracker &tracker_;
  /**
  * @brief camera transform with respect to body
  */
  tf::Transform camera_transform_;
  /**
  * @brief arm transform with respect to body
  */
  tf::Transform arm_transform_;
};
