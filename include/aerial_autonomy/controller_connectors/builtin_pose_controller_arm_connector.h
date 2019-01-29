#pragma once
#include "aerial_autonomy/controller_connectors/base_controller_connector.h"
#include "aerial_autonomy/controllers/builtin_controller.h"

#include <arm_parsers/arm_parser.h>

#include <tf/tf.h>

/**
 * @brief A visual servoing controller that uses a tracker output as feedback
 * and moves the arm to a goal pose relative to the tracked target
 */
class BuiltInPoseControllerArmConnector
    : public ControllerConnector<tf::Transform, tf::Transform, tf::Transform> {
public:
  /**
   * @brief Constructor
   */
  BuiltInPoseControllerArmConnector(ArmParser &arm_hardware,
                                    BuiltInPoseController &controller)
      : ControllerConnector(controller, ControllerGroup::Arm),
        arm_hardware_(arm_hardware) {}
  /**
   * @brief Destructor
   */
  virtual ~BuiltInPoseControllerArmConnector() {}

  void initialize();

protected:
  /**
   * @brief Extracts pose data from arm
   *
   * @param sensor_data Pose of arm end effector
   *
   * @return true if able to extract arm pose
   */
  virtual bool extractSensorData(tf::Transform &sensor_data);

  /**
   * @brief  Send position commands to hardware
   *
   * @param controls position command to send to arm
   */
  virtual void sendControllerCommands(tf::Transform controls);

private:
  /**
  * @brief Arm hardware to send commands
  */
  ArmParser &arm_hardware_;
};
