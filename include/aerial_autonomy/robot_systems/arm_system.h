#pragma once

// Html table writer
#include <aerial_autonomy/common/html_utils.h>
// Base robot system
#include <aerial_autonomy/robot_systems/base_robot_system.h>

#include <aerial_autonomy/controller_hardware_connectors/builtin_pose_controller_arm_connector.h>
#include <aerial_autonomy/controllers/builtin_controller.h>

// Arm hardware
#include <arm_parsers/arm_parser.h>

#include "arm_system_config.pb.h"

#include <iomanip>
#include <sstream>

/**
 * @brief Owns, initializes, and facilitates communication between different
 * hardware/software components.
 * Provides builtin set/get end effector pose, joint angles for a generic arm
*/
class ArmSystem : public virtual BaseRobotSystem {

public:
  /**
  * @brief Constructor
  *
  * ArmSystem requires an arm hardware. It instantiates the connectors,
  * controllers
  *
  * @param arm_hardware input hardware to send commands back
  */
  ArmSystem(ArmParser &arm_hardware, ArmSystemConfig config)
      : BaseRobotSystem(), arm_hardware_(arm_hardware),
        builtin_pose_controller_(config.pose_controller_config()),
        builtin_pose_controller_arm_connector_(arm_hardware_,
                                               builtin_pose_controller_) {
    controller_hardware_connector_container_.setObject(
        builtin_pose_controller_arm_connector_);
  }

  /**
  * @brief Public API call to get end effector transform
  */
  Eigen::Matrix4d getEndEffectorPose() {
    return arm_hardware_.getEndEffectorTransform();
  }

  /**
  * @brief Public API call to grip/ungrip an object
  *
  * @param grip_action true to grip an object and false to ungrip
  * @return True if command sent successfully, false otherwise
  */
  bool grip(bool grip_action) { return arm_hardware_.grip(grip_action); }

  /**
   * @brief Reset gripper state for passive grippers. For normal
   * grippers, it ungrips. This is usually called at the start
   * of a grasping application
   *
   * @return true if the gripper is reset successfully
   */
  bool resetGripper() { return arm_hardware_.resetGripper(); }

  /**
  * @brief Power the arm on/off
  *
  * @param state True to switch on, and False to switch off.
  */
  void power(bool state) {
    if (state) {
      arm_hardware_.sendCmd(ArmParser::POWER_ON);
    } else {
      arm_hardware_.sendCmd(ArmParser::POWER_OFF);
    }
  }

  /**
  * @brief Set the arm joints to a known folded configuration
  */
  void foldArm() { arm_hardware_.sendCmd(ArmParser::FOLD_ARM); }

  /**
  * @brief Set the arm joints to a known L shaped configuration.
  */
  void rightArm() { arm_hardware_.sendCmd(ArmParser::RIGHT_ARM); }

  /**
  * @brief Provide the current state of arm system
  * \todo Add status about arm powered on/off, and any error messages
  * can also add arm end effector pose if useful
  *
  * @return string representation of the arm system state
  */
  std::string getSystemStatus() const {
    HtmlTableWriter table_writer;
    table_writer.beginRow();
    table_writer.addHeader("Arm Status:", Colors::blue, 4);
    table_writer.beginRow();
    table_writer.addCell("Joint Angles: ");
    for (double q : arm_hardware_.getJointAngles()) {
      table_writer.addCell(q);
    }
    table_writer.beginRow();
    table_writer.addCell("Joint Velocities: ");
    for (double q : arm_hardware_.getJointVelocities()) {
      table_writer.addCell(q);
    }
    Eigen::Matrix4d ee_transform = arm_hardware_.getEndEffectorTransform();
    table_writer.beginRow();
    table_writer.addCell("End effector translation: ");
    for (int i = 0; i < 3; ++i) {
      table_writer.addCell(ee_transform(i, 3));
    }
    Eigen::Matrix3d ee_rotation = ee_transform.topLeftCorner(3, 3);
    Eigen::Vector3d euler_angles = ee_rotation.eulerAngles(2, 1, 0);
    table_writer.beginRow();
    table_writer.addCell("End effector RPY: ");
    for (int i = 0; i < 3; ++i) {
      table_writer.addCell(euler_angles[i]);
    }
    table_writer.beginRow();
    std::string command_status = (getCommandStatus() ? "True" : "False");
    std::string command_status_color =
        (getCommandStatus() ? Colors::green : Colors::red);
    table_writer.addCell(command_status, "CommandStatus", command_status_color,
                         2);
    table_writer.beginRow();
    std::string arm_enabled = (enabled() ? "True" : "False");
    std::string enabled_color = (enabled() ? Colors::green : Colors::red);
    table_writer.addCell(arm_enabled, "Enabled", enabled_color, 2);
    return table_writer.getTableString();
  }

  /**
  * @brief Verify the status of grip/power on/off and fold/rightArm commands
  *
  * @return True if the command is complete
  */
  bool getCommandStatus() const { return arm_hardware_.getCommandStatus(); }

  /**
  * @brief If arm is enabled return.
  *
  * @return True if arm enabled
  */
  bool enabled() const {
    return arm_hardware_.state == ArmParser::ENABLED ? true : false;
  }

private:
  /**
  * @brief Hardware
  */
  ArmParser &arm_hardware_;
  /**
  * @brief Controls the arm pose
  */
  BuiltInPoseController builtin_pose_controller_;
  /**
  * @brief Connects the arm controller to the arm hardware
  */
  BuiltInPoseControllerArmConnector builtin_pose_controller_arm_connector_;
};
