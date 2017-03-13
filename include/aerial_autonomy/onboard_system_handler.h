#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <parsernode/parser.h>
#include <pluginlib/class_loader.h>

#include "onboard_system_handler_config.pb.h"
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/common/async_timer.h>
#include <aerial_autonomy/robot_systems/uav_system.h>
#include <aerial_autonomy/state_machines/state_machine_gui_connector.h>
#include <aerial_autonomy/system_status_publisher.h>

/**
 * @brief Owns all of the autonomous system components and is responsible for
 * thread management.
 * @tparam LogicStateMachineT Logic state machine to use
 * @tparam EventManagerT Event manager to use
 */
template <class LogicStateMachineT, class EventManagerT>
class OnboardSystemHandler {
public:
  /**
   * @brief Constructor
   * @param nh NodeHandle to use for event and command subscription
   * @param config Proto configuration parameters
   */
  OnboardSystemHandler(ros::NodeHandle &nh, OnboardSystemHandlerConfig &config)
      : nh_(nh), config_(config),
        parser_loader(new pluginlib::ClassLoader<parsernode::Parser>(
            "parsernode", "parsernode::Parser")),
        uav_hardware_(
            parser_loader->createUnmanagedInstance(config_.uav_parser_type())),
        uav_system_(new UAVSystem(*uav_hardware_, config_.uav_system_config())),
        logic_state_machine_(new LogicStateMachineT(std::ref(*uav_system_))),
        event_manager_(new EventManagerT()),
        state_machine_gui_connector_(
            new StateMachineGUIConnector<EventManagerT, LogicStateMachineT>(
                nh_, std::ref(*event_manager_),
                std::ref(*logic_state_machine_))),
        system_status_pub_(new SystemStatusPublisher<LogicStateMachineT>(
            nh_, std::ref(*uav_system_), std::ref(*logic_state_machine_))),
        logic_state_machine_timer_(
            std::bind(&LogicStateMachineT::template process_event<
                          InternalTransitionEvent>,
                      std::ref(*logic_state_machine_),
                      InternalTransitionEvent()),
            std::chrono::milliseconds(config_.state_machine_timer_duration())),
        uav_controller_timer_(
            std::bind(&UAVSystem::runActiveController, std::ref(*uav_system_),
                      HardwareType::UAV),
            std::chrono::milliseconds(config_.uav_controller_timer_duration())),
        status_timer_(
            std::bind(
                &SystemStatusPublisher<LogicStateMachineT>::publishSystemStatus,
                std::ref(*system_status_pub_)),
            std::chrono::milliseconds(config_.status_timer_duration())) {
    // Initialize UAV plugin
    // TODO Gowtham: Make parser plugin throw exception if it cannot initialize
    uav_hardware_->initialize(nh);

    // Get the party started
    logic_state_machine_->start();
    logic_state_machine_timer_.start();
    uav_controller_timer_.start();
    status_timer_.start();
  }

  /**
  * @brief Delete copy constructor
  */
  OnboardSystemHandler(const OnboardSystemHandler &) = delete;

  /**
   * @brief Checks if internal ROS topics are connected
   * @return Returns true if connected and false otherwise
   */
  bool isConnected() {
    if (state_machine_gui_connector_) {
      return state_machine_gui_connector_->isEventManagerConnected() &&
             state_machine_gui_connector_->isPoseCommandConnected();
    } else {
      return false;
    }
  }

  /**
   * @brief Get UAV state
   * @return The UAV state
   */
  parsernode::common::quaddata getUAVData() {
    parsernode::common::quaddata quad_data;
    uav_hardware_->getquaddata(quad_data);
    return quad_data;
  }

private:
  ros::NodeHandle nh_; ///< ROS NodeHandle for processing events and commands
  OnboardSystemHandlerConfig config_; ///< Configuration parameters
  std::unique_ptr<pluginlib::ClassLoader<parsernode::Parser>>
      parser_loader; ///< Used to load hardware plugin
  std::unique_ptr<parsernode::Parser> uav_hardware_; ///< Hardware instance
  std::unique_ptr<UAVSystem> uav_system_;            ///< Contains controllers
  std::unique_ptr<LogicStateMachineT>
      logic_state_machine_; ///< State machine that gets run by the system
  std::unique_ptr<EventManagerT>
      event_manager_; ///< Event manager used by the state machine
  std::unique_ptr<StateMachineGUIConnector<EventManagerT, LogicStateMachineT>>
      state_machine_gui_connector_; ///< Connects event manager to the state
                                    /// machine
  std::unique_ptr<SystemStatusPublisher<LogicStateMachineT>>
      system_status_pub_;                ///< Publish quad status
  AsyncTimer logic_state_machine_timer_; ///< Timer for running state machine
  AsyncTimer uav_controller_timer_;      ///< Timer for running uav controller
  AsyncTimer status_timer_; ///< Update uav status and state machine status
};
