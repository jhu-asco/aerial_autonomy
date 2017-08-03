#pragma once

// Html table writer
#include <aerial_autonomy/common/html_table_writer.h>
// Base robot system
#include <aerial_autonomy/robot_systems/base_robot_system.h>
// Arm hardware
#include <arm_parsers/arm_parser.h>

#include <iomanip>
#include <sstream>

/**
 * @brief Owns, initializes, and facilitates communication between different
 * hardware/software components.
 * Provides builtin set/get end effector pose, joint angles for a generic arm
*/
class ArmSystem : public virtual BaseRobotSystem {

private:
  /**
  * @brief Hardware
  */
  ArmParser &arm_hardware_;

public:
  /**
  * @brief Constructor
  *
  * ArmSystem requires an arm hardware. It instantiates the connectors,
  * controllers
  *
  * @param arm_hardware input hardware to send commands back
  */
  ArmSystem(ArmParser &arm_hardware)
      : BaseRobotSystem(), arm_hardware_(arm_hardware) {}

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
  */
  void grip(bool grip_action) {
    if (grip_action) {
      //\todo Gowtham Change arm plugins to update gripping strategy
      int grip_position = 1000;
      arm_hardware_.grip(grip_position);
    } else {
      //\todo Gowtham Change arm plugins to update gripping strategy
      int grip_position = 2000;
      arm_hardware_.grip(grip_position);
    }
  }

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
    bool command_status = (getCommandStatus() ? "True" : "False");
    table_writer.addCell("CommandStatus: ", command_status);
    table_writer.beginRow();
    bool arm_enabled = (enabled() ? "True" : "False");
    table_writer.addCell("Enabled: ", arm_enabled);
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
};
