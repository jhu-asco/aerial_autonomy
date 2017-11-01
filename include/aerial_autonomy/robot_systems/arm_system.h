#pragma once

// Html table writer
#include <aerial_autonomy/common/html_utils.h>
// Base robot system
#include <aerial_autonomy/robot_systems/base_robot_system.h>

#include <aerial_autonomy/controller_hardware_connectors/builtin_pose_controller_arm_connector.h>
#include <aerial_autonomy/controllers/builtin_controller.h>

// Arm hardware
#include <arm_parsers/arm_parser.h>
#include <arm_parsers/arm_simulator.h>
#include <arm_parsers/generic_arm.h>
#include <arm_parsers/simple_arm.h>

#include "arm_system_config.pb.h"

#include <memory>

#include <iomanip>
#include <sstream>

/**
 * @brief Owns, initializes, and facilitates communication between different
 * hardware/software components.
 * Provides builtin set/get end effector pose, joint angles for a generic arm
*/
class ArmSystem : public virtual BaseRobotSystem {
protected:
  using ArmParserPtr = std::shared_ptr<ArmParser>;

public:
  /**
  * @brief Constructor with default config
  * @param arm_hardware Arm hardware driver
  */
  ArmSystem(ArmParserPtr arm_hardware)
      : ArmSystem(ArmSystemConfig(), arm_hardware) {}

  /**
  * @brief Constructor
  *
  * ArmSystem requires an arm hardware. It instantiates the connectors,
  * controllers
  *
  * @param config The configuration file for arm system parameters
  * @param arm_hardware input hardware to send commands back. If provided,
  * overwrites the
  * one in the config file
  */
  ArmSystem(ArmSystemConfig config, ArmParserPtr arm_hardware = nullptr)
      : BaseRobotSystem(),
        arm_hardware_(ArmSystem::chooseArmHardware(arm_hardware, config)),
        builtin_pose_controller_(config.pose_controller_config()),
        builtin_pose_controller_arm_connector_(*arm_hardware_,
                                               builtin_pose_controller_) {
    controller_hardware_connector_container_.setObject(
        builtin_pose_controller_arm_connector_);
  }

  /**
  * @brief Public API call to get end effector transform
  */
  Eigen::Matrix4d getEndEffectorPose() {
    return arm_hardware_->getEndEffectorTransform();
  }

  /**
  * @brief Public API call to grip/ungrip an object
  *
  * @param grip_action true to grip an object and false to ungrip
  * @return True if command sent successfully, false otherwise
  */
  bool grip(bool grip_action) { return arm_hardware_->grip(grip_action); }

  /**
   * @brief Reset gripper state for passive grippers. For normal
   * grippers, it ungrips. This is usually called at the start
   * of a grasping application
   *
   * @return true if the gripper is reset successfully
   */
  bool resetGripper() { return arm_hardware_->resetGripper(); }

  /**
  * @brief Power the arm on/off
  *
  * @param state True to switch on, and False to switch off.
  */
  void power(bool state) {
    if (state) {
      arm_hardware_->sendCmd(ArmParser::POWER_ON);
    } else {
      arm_hardware_->sendCmd(ArmParser::POWER_OFF);
    }
  }

  /**
  * @brief Set the arm joints to a known folded configuration
  */
  void foldArm() { arm_hardware_->sendCmd(ArmParser::FOLD_ARM); }

  /**
  * @brief Set the arm joints to a known L shaped configuration.
  */
  void rightArm() { arm_hardware_->sendCmd(ArmParser::RIGHT_ARM); }

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
    for (double q : arm_hardware_->getJointAngles()) {
      table_writer.addCell(q);
    }
    table_writer.beginRow();
    table_writer.addCell("Joint Velocities: ");
    for (double q : arm_hardware_->getJointVelocities()) {
      table_writer.addCell(q);
    }
    Eigen::Matrix4d ee_transform = arm_hardware_->getEndEffectorTransform();
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
  bool getCommandStatus() const { return arm_hardware_->getCommandStatus(); }

  /**
  * @brief If arm is enabled return.
  *
  * @return True if arm enabled
  */
  bool enabled() const {
    return arm_hardware_->state() == ArmParser::ENABLED ? true : false;
  }

protected:
  /**
  * @brief Hardware driver
  */
  ArmParserPtr arm_hardware_;
  /**
   * @brief choose between user provided parser and the one specified in config
   * file
   *
   * @param parser if user provided a parser which is not null, that is chosen.
   * @param config if user provided null, the one specified in this config is
   * used.
   *
   * @return the chosen parser object
   */
  static ArmParserPtr chooseArmHardware(ArmParserPtr parser,
                                        ArmSystemConfig &config) {
    ArmParserPtr arm_parser_pointer;
    if (parser) {
      arm_parser_pointer = parser;
    } else {
      std::string arm_parser_type = config.arm_parser_type();
      if (arm_parser_type == "GenericArm") {
        arm_parser_pointer = ArmParserPtr(new GenericArm());
      } else if (arm_parser_type == "SimpleArm") {
        arm_parser_pointer = ArmParserPtr(new SimpleArm());
      } else if (arm_parser_type == "ArmSimulator") {
        arm_parser_pointer = ArmParserPtr(new ArmSimulator());
      } else {
        throw std::runtime_error("Unknown arm parser type provided: " +
                                 arm_parser_type);
      }
    }
    return arm_parser_pointer;
  }

private:
  /**
  * @brief Controls the arm pose
  */
  BuiltInPoseController builtin_pose_controller_;
  /**
  * @brief Connects the arm controller to the arm hardware
  */
  BuiltInPoseControllerArmConnector builtin_pose_controller_arm_connector_;
};
