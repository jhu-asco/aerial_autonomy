#pragma once

#include <ros/ros.h>

#include <parsernode/parser.h>

#include <aerial_autonomy/common/async_timer.h>
#include <aerial_autonomy/robot_systems/uav_system.h>
#include <aerial_autonomy/state_machines/state_machine_gui_connector.h>
#include <aerial_autonomy/tests/sample_parser.h>

/**
 * @brief Owns all of the autonomous system components and is responsible for
 * thread management.
 * @tparam LogicStateMachineT Logic state machine to use
 * @tparam EventManagerT Event manager to use
 */
template <class LogicStateMachineT, class EventManagerT>
class OnboardSystemHandler {
public:
  OnboardSystemHandler(ros::NodeHandle &nh)
      : nh_(nh),
        logic_state_machine_timer_(
            std::bind(&OnboardSystemHandler::stateMachineThread, this),
            std::chrono::milliseconds(50)),
        drone_controller_timer_(
            std::bind(&OnboardSystemHandler::droneControllerThread, this),
            std::chrono::milliseconds(50)) {
    // Get proto filepath from ROS params
    // Load config proto
    // Load configured drone parser
    drone_hardware_.reset(new SampleParser());
    uav_system_.reset(new UAVSystem(*drone_hardware_));
    logic_state_machine_.reset(
        new LogicStateMachineT(boost::ref(*uav_system_)));
    event_manager_.reset(new EventManagerT());
    state_machine_gui_connector_.reset(
        new StateMachineGUIConnector<EventManagerT, LogicStateMachineT>(
            nh_, boost::ref(*event_manager_),
            boost::ref(*logic_state_machine_)));
    logic_state_machine_->start();
    logic_state_machine_timer_.start();
    drone_controller_timer_.start();
  }

  OnboardSystemHandler(const OnboardSystemHandler &) = delete;

private:
  /**
   * @brief State machine processing loop
   */
  void stateMachineThread() {
    logic_state_machine_->process_event(InternalTransitionEvent());
  }

  /**
   * @brief Drone control loop
   */
  void droneControllerThread() {
    uav_system_->runActiveController(HardwareType::UAV);
  }

  ros::NodeHandle nh_;
  std::unique_ptr<parsernode::Parser> drone_hardware_;
  std::unique_ptr<UAVSystem> uav_system_;
  std::unique_ptr<LogicStateMachineT> logic_state_machine_;
  std::unique_ptr<EventManagerT> event_manager_;
  std::unique_ptr<StateMachineGUIConnector<EventManagerT, LogicStateMachineT>>
      state_machine_gui_connector_;
  AsyncTimer logic_state_machine_timer_;
  AsyncTimer drone_controller_timer_;
};
