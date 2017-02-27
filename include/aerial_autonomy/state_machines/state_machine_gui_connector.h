#pragma once
#include <ros/ros.h>
#include <std_msgs/String.h>

template <class EventManagerT, class LogicStateMachineT>
class StateMachineGUIConnector {
public:
  StateMachineGUIConnector(ros::NodeHandle &nh, EventManagerT &event_manager,
                           LogicStateMachineT &logic_state_machine)
      : event_manager_(event_manager),
        logic_state_machine_(logic_state_machine) {
    event_manager_sub_ = nh.subscribe(
        "event_manager", 1, &StateMachineGUIConnector::eventCallback, this);
  }

private:
  void eventCallback(const std_msgs::StringConstPtr &event_data) {
    event_manager_.triggerEvent(event_data->data, logic_state_machine_);
  }

  ros::Subscriber event_manager_sub_;
  EventManagerT &event_manager_;
  LogicStateMachineT &logic_state_machine_;
};
