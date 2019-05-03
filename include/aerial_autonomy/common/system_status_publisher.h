#pragma once

#include <ros/ros.h>

// Html utils
#include <aerial_autonomy/common/html_utils.h>
// controller group
#include <aerial_autonomy/types/controller_groups.h>
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
    VLOG(1) << "System Status Publisher Constructor";//TAGGED
  }

  /**
  * @brief Publish system status and state machine status
  * \todo Replace status text with html script. Need a html manager
  * to automatically add table lines
  */
  void publishSystemStatus() {
    // Get active controller status
    ControllerStatus uav_controller_status =
        robot_system_.getActiveControllerStatus(ControllerGroup::UAV);
    ControllerStatus arm_controller_status =
        robot_system_.getActiveControllerStatus(ControllerGroup::Arm);
    std::string robot_system_status = robot_system_.getSystemStatus();
    std::string current_state_name = pstate(logic_state_machine_);
    std::string no_transition_event_name =
        logic_state_machine_.get_no_transition_event_index().name();
    std::type_index last_transition_event_index =
        logic_state_machine_.lastProcessedEventIndex();
    HtmlDivisionWriter division_writer;
    division_writer.addHeader("Robot System Status");
    division_writer.addText(robot_system_status);
    // Add subheader for uav controller status
    division_writer.addHeader("UAV Controller Status", 4);
    division_writer.addText(uav_controller_status.getHtmlStatusString());
    // Add subheader for arm controller status
    division_writer.addHeader("Arm Controller Status", 4);
    division_writer.addText(arm_controller_status.getHtmlStatusString());
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
    logic_state_machine_table.beginRow();
    logic_state_machine_table.addCell("Last Event: ");
    if (last_transition_event_index == typeid(be::Abort)) {
      logic_state_machine_table.addCell(last_transition_event_index.name(), "",
                                        Colors::red);
    } else if (last_transition_event_index == typeid(Completed)) {
      logic_state_machine_table.addCell(last_transition_event_index.name(), "",
                                        Colors::green);
    } else {
      logic_state_machine_table.addCell(last_transition_event_index.name());
    }
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
