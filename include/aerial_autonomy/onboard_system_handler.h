#pragma once

#include <ros/ros.h>

#include <boost/thread.hpp>

#include <parsernode/parser.h>

#include <aerial_autonomy/robot_systems/uav_system.h>
#include <aerial_autonomy/state_machines/basic_state_machine.h>

/**
 * @brief Owns all of the autonomous system components and is responsible for
 * thread management.
 */
template <class LogicStateMachineT, EventManagerT> class OnboardSystemHandler {
public:
  OnboardSystemHandler(ros::NodeHandle &nh)
      : nh_(nh), uav_system_(drone_hardware_),
        logic_state_machine_(uav_system_), event_manager_(logic_state_machine_),
        state_machine_gui_connector<EventManagerT>(nh_, event_manager_) {
    // Get proto filepath from ROS params
    // Load config proto
    // Load configured drone parser
    // logic_state_machine_ = new LogicStateMachine(uav_system_);
    // event_manager_ = new BasicEventManager(*logic_state_machine_);
    // state_machine_gui_connector =
    //    new StateMachineGUIConnector<BasicEventManager>(nh_, *event_manager_);

    boost::thread { stateMachineThread }
    boost::thread { droneControllerThread }
  }

private:
  void stateMachineThread() {
    // do on timer
    logic_state_machine_.process_event(InternalTransitionEvent());
  }

  void droneControllerThread() {
    // do on timer
    uav_system_.runActiveController(HardwareType::UAV);
  }

  ros::NodeHandle nh_;
  parsernode::Parser drone_hardware_;
  UAVSystem uav_system_;
  LogicStateMachineT logic_state_machine_;
  EventManagerT event_manager_;
};
