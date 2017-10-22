#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "common_system_handler_config.pb.h"
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/common/async_timer.h>
#include <aerial_autonomy/common/system_status_publisher.h>
#include <aerial_autonomy/state_machines/state_machine_gui_connector.h>

/**
 * @brief Provides logic common to different system handlers to
 * reduce code duplication
 * @tparam LogicStateMachineT Logic state machine to use
 * @tparam EventManagerT Event manager to use
 * @tparam RobotSystemT robot system used by the state machine
 */
template <class LogicStateMachineT, class EventManagerT, class RobotSystemT>
class CommonSystemHandler {
public:
  /**
   * @brief Constructor
   *
   * Instantiates state machine, timers. Individual system handlers must
   * call startTimers to start the state machine and status timers.
   *
   * A convenience function isConnected is provided to check if the ros topics
   * associated with state machine event processing are connected or not. The
   * individual system handlers must extend this function if needed.
   *
   * @param nh NodeHandle to use for event and command subscription
   * @param config Proto configuration parameters
   * @param robot_system robot system used to create logic state machine
   */
  CommonSystemHandler(const CommonSystemHandlerConfig &config,
                      RobotSystemT &robot_system,
                      const BaseStateMachineConfig &state_machine_config)
      : nh_("~common"), logic_state_machine_(std::ref(robot_system),
                                             std::cref(state_machine_config)),
        state_machine_gui_connector_(nh_, event_manager_, logic_state_machine_),
        system_status_pub_(nh_, robot_system, logic_state_machine_),
        status_timer_(
            std::bind(
                &SystemStatusPublisher<LogicStateMachineT>::publishSystemStatus,
                std::ref(system_status_pub_)),
            std::chrono::milliseconds(config.status_timer_duration())),
        logic_state_machine_timer_(
            std::bind(&LogicStateMachineT::template process_event<
                          InternalTransitionEvent>,
                      std::ref(logic_state_machine_),
                      InternalTransitionEvent()),
            std::chrono::milliseconds(config.state_machine_timer_duration())) {}

  /**
  * @brief Delete copy constructor
  */
  CommonSystemHandler(const CommonSystemHandler &) = delete;

  /**
   * @brief Checks if internal ROS topics are connected
   * @return Returns true if connected and false otherwise
   */
  bool isConnected() {
    return state_machine_gui_connector_.isEventManagerConnected() &&
           state_machine_gui_connector_.isPoseCommandConnected();
  }

  /**
  * @brief Start state machine internal event processing and status timer
  */
  void startTimers() {
    logic_state_machine_.start();
    logic_state_machine_timer_.start();
    status_timer_.start();
  }

protected:
  ros::NodeHandle nh_;
  LogicStateMachineT
      logic_state_machine_;     ///< State machine that gets run by the system
  EventManagerT event_manager_; ///< Event manager used by the state machine
  StateMachineGUIConnector<EventManagerT, LogicStateMachineT>
      state_machine_gui_connector_; ///< Connects event manager to the state
                                    /// machine
  SystemStatusPublisher<LogicStateMachineT>
      system_status_pub_;   ///< publishes status messages
  AsyncTimer status_timer_; ///< Update uav status and state machine status
  AsyncTimer logic_state_machine_timer_; ///< Timer for running state machine
};
