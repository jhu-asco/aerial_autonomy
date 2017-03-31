#pragma once
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/logic_states/base_state.h>
/// \todo Matt: Change this to correct robot system
#include <aerial_autonomy/robot_systems/uav_system.h>
///
#include <aerial_autonomy/types/completed_event.h>
#include <glog/logging.h>
#include <parsernode/common.h>

/**
* @brief
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct VisualServoingTransitionActionFunctor_
    : EventAgnosticActionFunctor<UAVSystem, LogicStateMachineT> {
  void run(UAVSystem &robot_system, LogicStateMachineT &) {
    VLOG(1) << "Selecting Home Location";
    robot_system.setHomeLocation();
    /// \todo Matt: Select the right visual servoing controller for quadrotor
  }
};

/**
* @brief
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct VisualServoingAbortActionFunctor_
    : EventAgnosticActionFunctor<UAVSystem, LogicStateMachineT> {
  void run(UAVSystem &robot_system, LogicStateMachineT &) {
    LOG(WARNING) << "Aborting Visual servoing Controller";
    robot_system.abortController(HardwareType::UAV);
  }
};

/**
* @brief Action to reach a pre designated point
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct GoHomeTransitionActionFunctor_
    : EventAgnosticActionFunctor<UAVSystem, LogicStateMachineT> {
  void run(UAVSystem &robot_system, LogicStateMachineT &) {
    PositionYaw home_location = robot_system.getHomeLocation();
    VLOG(1) << "Going home";
    robot_system.setGoal<PositionControllerDroneConnector, PositionYaw>(
        home_location);
  }
};

template <class LogicStateMachineT>
struct GoHomeTransitionGuardFunctor_
    : EventAgnosticGuardFunctor<UAVSystem, LogicStateMachineT> {
  bool guard(UAVSystem &robot_system, LogicStateMachineT &) {
    return robot_system.isHomeLocationSpecified();
  }
};

/**
* @brief Logic to check while reaching a visual servoing goal
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct VisualServoingInternalActionFunctor_
    : EventAgnosticActionFunctor<UAVSystem, LogicStateMachineT> {
  /**
  * @brief check if we reached VS goal and trigger completed event
  *
  * @param robot_system robot system to get sensor data
  * @param logic_state_machine logic state machine to trigger events
  */
  virtual void run(UAVSystem &robot_system,
                   LogicStateMachineT &logic_state_machine) {
    /// \todo Matt: Get current goal for visual servoing
    /// //Example:
    /// PositionYaw goal =
    ///    robot_system.getGoal<PositionControllerDroneConnector,
    ///    PositionYaw>();
    /// \todo Matt: Implement the correct function to check if we
    /// reached the visual servoing goal based on object distance.
    /// Also need to check if tracking is valid
    PositionYaw goal;
    // Get current position, yaw
    parsernode::common::quaddata data = robot_system.getUAVData();
    geometry_msgs::Vector3 current_position = data.localpos;
    double yaw = data.rpydata.z;
    // Define tolerance and check if reached goal
    const auto &robot_config = robot_system.getConfiguration();
    const double &tolerance_pos = robot_config.goal_position_tolerance(); // m
    const double &tolerance_yaw = robot_config.goal_yaw_tolerance();      // m
    if (data.batterypercent < robot_config.minimum_battery_percent()) {
      LOG(WARNING) << "Battery too low " << data.batterypercent
                   << "\% Landing!";
      logic_state_machine.process_event(be::Land());
    } else if (std::abs(current_position.x - goal.x) < tolerance_pos &&
               std::abs(current_position.y - goal.y) < tolerance_pos &&
               std::abs(current_position.z - goal.z) < tolerance_pos &&
               std::abs(yaw - goal.yaw) < tolerance_yaw) {
      VLOG(1) << "Reached goal";
      logic_state_machine.process_event(Completed());
    }
  }
};

template <class LogicStateMachineT>
struct VisualServoingTransitionGuardFunctor_
    : EventAgnosticGuardFunctor<UAVSystem, LogicStateMachineT> {
  bool guard(UAVSystem &robot_system_, LogicStateMachineT &) {
    ///\todo Matt: Implement function to check if tracking is valid
    return true;
  }
};

/**
* @brief State that uses position control functor to reach a desired goal.
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using VisualServoing_ =
    BaseState<UAVSystem, LogicStateMachineT,
              VisualServoingInternalActionFunctor_<LogicStateMachineT>>;
