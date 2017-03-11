#pragma once
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/basic_events.h>
#include <aerial_autonomy/controller_hardware_connectors/position_controller_drone_connector.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <aerial_autonomy/robot_systems/uav_system.h>
#include <aerial_autonomy/types/completed_event.h>
#include <parsernode/common.h>

namespace be = basic_events;

/**
* @brief Transition action to perform when going into position control mode
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct PositionControlTransitionActionFunctor_
    : ActionFunctor<PositionYaw, UAVSystem, LogicStateMachineT> {
  void run(const PositionYaw &goal, UAVSystem &robot_system,
           LogicStateMachineT &) {
    robot_system.setGoal<PositionControllerDroneConnector, PositionYaw>(goal);
  }
};

/**
* @brief Transition action to perform when aborting position control
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct PositionControlAbortActionFunctor_
    : EventAgnosticActionFunctor<UAVSystem, LogicStateMachineT> {
  void run(UAVSystem &robot_system, LogicStateMachineT &) {
    robot_system.abortController(HardwareType::UAV);
  }
};

/**
* @brief Guard function to check the goal is within tolerance before starting
* towards goal
*
* \todo Use a parameter for setting position tolerance
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct PositionControlTransitionGuardFunctor_
    : GuardFunctor<PositionYaw, UAVSystem, LogicStateMachineT> {
  bool guard(const PositionYaw &goal, UAVSystem &robot_system,
             LogicStateMachineT &) {
    parsernode::common::quaddata data = robot_system.getUAVData();
    geometry_msgs::Vector3 current_position = data.localpos;
    // Check goal is close to position before sending goal (Can use a geofence
    // here)
    // Ensure the goal is within 100 meters of the current position
    // TODO Use a parameter for setting position tolerance
    double tolerance_pos = 100;
    bool result = true;
    if (std::abs(current_position.x - goal.x) > tolerance_pos ||
        std::abs(current_position.y - goal.y) > tolerance_pos ||
        std::abs(current_position.z - goal.z) > tolerance_pos) {
      result = false;
    }
    return result;
  }
};

/**
* @brief Logic to check while reaching a position control goal
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct PositionControlInternalActionFunctor_
    : EventAgnosticActionFunctor<UAVSystem, LogicStateMachineT> {
  /**
  * @brief check if we reached goal and trigger hovering if we reached goal.
  *
  * @param robot_system robot system to get sensor data
  * @param logic_state_machine logic state machine to trigger events
  */
  virtual void run(UAVSystem &robot_system,
                   LogicStateMachineT &logic_state_machine) {
    // Get current goal
    PositionYaw goal =
        robot_system.getGoal<PositionControllerDroneConnector, PositionYaw>();
    // Get current position, yaw
    parsernode::common::quaddata data = robot_system.getUAVData();
    geometry_msgs::Vector3 current_position = data.localpos;
    double yaw = data.rpydata.z;
    // Define tolerance and check if reached goal
    // TODO Use a parameter for setting position tolerance
    double tolerance_pos = 1.0; // m
    double tolerance_yaw = 0.1; // rad
    if (data.batterypercent <
        robot_system.getConfiguration().minimum_battery_percent()) {
      logic_state_machine.process_event(be::Land());
    } else if (std::abs(current_position.x - goal.x) < tolerance_pos &&
               std::abs(current_position.y - goal.y) < tolerance_pos &&
               std::abs(current_position.z - goal.z) < tolerance_pos &&
               std::abs(yaw - goal.yaw) < tolerance_yaw) {
      // Reached goal
      logic_state_machine.process_event(Completed());
    }
  }
};

/**
* @brief State that uses position control functor to reach a desired goal.
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using ReachingGoal_ =
    BaseState<UAVSystem, LogicStateMachineT,
              PositionControlInternalActionFunctor_<LogicStateMachineT>>;
