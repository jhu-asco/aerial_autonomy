#pragma once
#include <ros/ros.h>
#include <std_msgs/String.h>

template <EventManagerT> class StatemMachineGUIConnector {
  StatemMachineGUIConnector(ros::NodeHandle &nh, EventManagerT &event_manager)
      : event_manager_(event_manager) {
    nh.subscribe("event_manager", 1, StatemMachineGUIConnector::eventCallback,
                 this);
  }
  void eventCallback(const std_msgs::StringConstPtr &event_data) {
    event_manager_.triggerEvent(event_data->data);
  }
  EventManagerT &event_manager_;
};
