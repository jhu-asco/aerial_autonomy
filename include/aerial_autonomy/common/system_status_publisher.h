#pragma once

#include <ros/ros.h>

// Html utils
#include <aerial_autonomy/common/html_utils.h>
// Base Robot system
#include <aerial_autonomy/robot_systems/base_robot_system.h>

/**
 * @brief Responsible for publishing system status message
 * @tparam Logic state machine type that we are getting status from
 */
template <class LogicStateMachineT> class SystemStatusPublisher {
public:
  /**
   * @brief Constructor
   * @param nh NodeHandle to use for publishing system status
   * @param robot_system RobotSystem whose status will be published
   * @param logic_state_machine State machine whose status will be published
   */
  SystemStatusPublisher(ros::NodeHandle &nh,
                        const BaseRobotSystem &robot_system,
                        LogicStateMachineT &logic_state_machine)
      : nh_(nh),
        system_status_pub_(nh.advertise<std_msgs::String>("system_status", 1)),
        robot_system_(robot_system), logic_state_machine_(logic_state_machine) {
  }

  /**
  * @brief Publish system status and state machine status
  * \todo Replace status text with html script. Need a html manager
  * to automatically add table lines
  */
  void publishSystemStatus() {
    ControllerStatus controller_status;
    HtmlTableWriter controller_status_table;
    controller_status_table.beginRow();
    controller_status_table.addHeader("Controller Status", Colors::blue, 2);
    controller_status_table.beginRow();
    if (robot_system_.getActiveControllerStatus(HardwareType::UAV,
                                                controller_status)) {
      switch (controller_status) {
      case ControllerStatus::Active:
        controller_status_table.addCell("Active", "Status", Colors::green);
        break;
      case ControllerStatus::Completed:
        controller_status_table.addCell("Completed", "Status", Colors::green);
        break;
      case ControllerStatus::Critical:
        controller_status_table.addCell("Critical", "Status", Colors::red);
        break;
      default:
        controller_status_table.addCell("Unknown controller status!", "Status",
                                        Colors::red, 2);
        break;
      }
    } else {
      controller_status_table.addCell("Unknown controller status!", "Status",
                                      Colors::yellow, 2);
    }
    std::string robot_system_status = robot_system_.getSystemStatus();
    std::string current_state_name = pstate(logic_state_machine_);
    std::string no_transition_event_name =
        logic_state_machine_.get_no_transition_event_index().name();
    HtmlDivisionWriter division_writer;
    division_writer.addHeader("Robot System Status");
    division_writer.addText(robot_system_status);
    division_writer.addText(controller_status_table.getTableString());
    // add table for logic state machine
    HtmlTableWriter logic_state_machine_table(190);
    logic_state_machine_table.beginRow();
    logic_state_machine_table.addHeader("Logic State Machine Status",
                                        Colors::blue, 2);
    logic_state_machine_table.beginRow();
    logic_state_machine_table.addCell("Current state: ");
    logic_state_machine_table.addCell(current_state_name);
    logic_state_machine_table.beginRow();
    logic_state_machine_table.addCell("Last Event without transition: ");
    logic_state_machine_table.addCell(no_transition_event_name);
    // Add table to division
    division_writer.addText(logic_state_machine_table.getTableString());
    std_msgs::String status;
    status.data = division_writer.getDivisionText();
    system_status_pub_.publish(status);
  }

private:
  ros::NodeHandle &nh_;              ///< NodeHandle used for publishing
  ros::Publisher system_status_pub_; ///< publishes status messages
  const BaseRobotSystem
      &robot_system_; ///< system whose status we are publishing
  const LogicStateMachineT
      &logic_state_machine_; ///< state machine whose status we are publishing
};
