#pragma once

#include <ros/ros.h>

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
    std::string controller_status_str;
    if (robot_system_.getActiveControllerStatus(HardwareType::UAV,
                                                controller_status)) {
      switch (controller_status) {
      case ControllerStatus::Active:
        controller_status_str = "Active";
        break;
      case ControllerStatus::Completed:
        controller_status_str = "Completed";
        break;
      case ControllerStatus::Critical:
        controller_status_str = "Critical";
        break;
      default:
        controller_status_str = "Unknown controller status!";
        break;
      }
    } else {
      controller_status_str = "No active controller";
    }
    std::string robot_system_status = robot_system_.getSystemStatus();
    std::string current_state_name = pstate(logic_state_machine_);
    std::string no_transition_event_name =
        logic_state_machine_.get_no_transition_event_index().name();
    std_msgs::String status;
    status.data = "Robot System Status:\n" + robot_system_status + "\n";
    status.data += "\n\n========================\n\n";
    status.data += "Controller Status: " + controller_status_str;
    status.data += "\n\n========================\n\n";
    status.data += "Logic State Machine Status: \n";
    status.data += "Current state:\t" + current_state_name + "\n";
    status.data +=
        "Last event without transition:\t" + no_transition_event_name + "\n";
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
