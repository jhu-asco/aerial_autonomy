#pragma once

// Base robot system
#include <aerial_autonomy/robot_systems/base_robot_system.h>
// Arm hardware
#include <arm_parsers/generic_arm.h>
/// \todo Add controllers and controller connectors for visual servoing

/**
 * @brief Owns, initializes, and facilitates communication between different
 * hardware/software components.
 * Provides builtin set/get end effector pose, joint angles for a generic arm
*/
class ArmSystem : public BaseRobotSystem {

private:
  /**
  * @brief Hardware
  */
  GenericArm arm_hardware_;
  // \todo Add controllers, controller connectors, config if needed

public:
  /**
  * @brief Constructor
  *
  * ArmSystem requires an arm hardware. It instantiates the connectors,
  * controllers
  *
  * @param arm_hardware input hardware to send commands back
  */
  ArmSystem(ros::NodeHandle &nh) : BaseRobotSystem(), arm_hardware_(nh) {}

  /**
  * @brief Public API call to set end effector transform
  */
  void setEndEffectorPose(const Eigen::Matrix4d &end_effector_pose) {
    arm_hardware_.setEndEffectorPose(end_effector_pose);
  }

  /**
  * @brief Public API call to get end effector transform
  */
  Eigen::Matrix4d getEndEffectorPose() {
    return arm_hardware_.getEndEffectorTransform();
  }

  /**
  * @brief Public API call to set gripper position
  *
  * @param grip_position Grip position in PWM to set
  */
  void grip(double grip_position) { arm_hardware_.grip(grip_position); }

  /**
  * @brief Power the arm on/off
  *
  * @param state True to switch on, and False to switch off.
  */
  void powerOnOff(bool state) {
    if (state) {
      arm_hardware_.sendCmd("power on");
    } else {
      arm_hardware_.sendCmd("power off");
    }
  }

  /**
  * @brief Set the arm joints to a known folded configuration
  */
  void gotoFoldArmConfiguration() { arm_hardware_.sendCmd("fold arm"); }

  /**
  * @brief Set the arm joints to a known L shaped configuration.
  */
  void gotoRightArmConfiguration() { arm_hardware_.sendCmd("right arm"); }

  /**
  * @brief Provide the current state of UAV system
  * \todo Add status about arm powered on/off, and any error messages
  * can also add arm end effector pose if useful
  *
  * @return string representation of the UAV system state
  */
  std::string getSystemStatus() const { return std::string(); }
};
