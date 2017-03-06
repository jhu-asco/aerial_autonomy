#pragma once

#include <ros/ros.h>

#include <parsernode/parser.h>
#include <pluginlib/class_loader.h>

#include "onboard_system_handler_config.pb.h"
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
  OnboardSystemHandler(ros::NodeHandle &nh, OnboardSystemHandlerConfig &config)
      : nh_(nh), logic_state_machine_timer_(
                     std::bind(&OnboardSystemHandler::stateMachineThread, this),
                     std::chrono::milliseconds(50)),
        drone_controller_timer_(
            std::bind(&OnboardSystemHandler::droneControllerThread, this),
            std::chrono::milliseconds(50)),
        config_(config) {
    // Load configured drone parser
    loadUAVPlugin(nh_, config_.uav_parser_type());

    // Instantiate members
    drone_hardware_.reset(new SampleParser());
    uav_system_.reset(new UAVSystem(*drone_hardware_));
    logic_state_machine_.reset(
        new LogicStateMachineT(boost::ref(*uav_system_)));
    event_manager_.reset(new EventManagerT());
    state_machine_gui_connector_.reset(
        new StateMachineGUIConnector<EventManagerT, LogicStateMachineT>(
            nh_, boost::ref(*event_manager_),
            boost::ref(*logic_state_machine_)));

    // Get the party started
    logic_state_machine_->start();
    logic_state_machine_timer_.start();
    drone_controller_timer_.start();
  }

  OnboardSystemHandler(const OnboardSystemHandler &) = delete;

private:
  /**
   * @brief Load the UAV plugin with the given name
   * @param nh NodeHandle for the plugin to use for ROS communication
   * @param plugin_name Name of plugin to load
   */
  bool loadUAVPlugin(ros::NodeHandle &nh, std::string plugin_name) {
    // TODO(matt): Maybe move this functionality to its own class
    if (!parser_loader)
      parser_loader.reset(new pluginlib::ClassLoader<parsernode::Parser>(
          "parsernode", "parsernode::Parser"));

    try {
      drone_hardware_ = parser_loader->createInstance(plugin_name);
      drone_hardware_->initialize(nh);
    } catch (pluginlib::PluginlibException &ex) {
      std::cout << "The plugin failed to load: " << ex.what() << std::endl;
      return false;
    }
    // Wait till parser is initialized:
    auto load_time = std::chrono::steady_clock::now();
    while (std::chrono::seconds(std::chrono::steady_clock::now() - load_time)
               .count() < config_.uav_parser_load_timeout()) {
      if (drone_hardware_->initialized)
        break;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return drone_hardware_->initialized;
  }
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
  std::unique_ptr<pluginlib::ClassLoader<parsernode::Parser>> parser_loader;
  AsyncTimer logic_state_machine_timer_;
  AsyncTimer drone_controller_timer_;
  OnboardSystemHandlerConfig config_;
};
