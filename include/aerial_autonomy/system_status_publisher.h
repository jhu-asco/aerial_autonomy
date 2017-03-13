#pragma once

#include <ros/ros.h>

#include <aerial_autonomy/robot_systems/uav_system.h>

/**
 * @brief Responsible for publishing system status message
 * @tparam Logic state machine type that we are getting status from
 */
template <class LogicStateMachineT> class SystemStatusPublisher {
public:
  /**
   * @brief Constructor
   * @param nh NodeHandle to use for publishing system status
   * @param uav_system UAVSystem whose status will be published
   * @param logic_state_machine State machine whose status will be published
   */
  SystemStatusPublisher(ros::NodeHandle &nh, UAVSystem &uav_system,
                        LogicStateMachineT &logic_state_machine)
      : nh_(nh),
        system_status_pub_(nh.advertise<std_msgs::String>("system_status", 1)),
        uav_system_(uav_system), logic_state_machine_(logic_state_machine) {}

  /**
  * @brief Publish system status and state machine status
  * \todo Replace status text with html script. Need a html manager
  * to automatically add table lines
  */
  void publishSystemStatus() {
    std::string uav_system_status = uav_system_.getSystemStatus();
    std::string current_state_name = pstate(logic_state_machine_);
    std::string no_transition_event_name =
        logic_state_machine_.get_no_transition_event_index().name();
    std_msgs::String status;
    status.data = "UAV System Status:\n" + uav_system_status + "\n";
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
  UAVSystem &uav_system_;            ///< system whose status we are publishing
  LogicStateMachineT
      &logic_state_machine_; ///< state machine whose status we are publishing
};
