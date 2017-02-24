#pragma once
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <aerial_autonomy/robot_systems/quadrotor_system.h>
#include <aerial_autonomy/basic_events.h>
#include <aerial_autonomy/types/completed_event.h>
#include <aerial_autonomy/controller_hardware_connectors/position_controller_drone_connector.h>
#include <parsernode/common.h>

using namespace basic_events;

template <class LogicStateMachineT>
struct PositionControlTransitionActionFunctor_
    : ActionFunctor<PositionYaw, QuadRotorSystem, LogicStateMachineT> {
  void run(const PositionYaw &goal, QuadRotorSystem &robot_system,
           LogicStateMachineT &) {
    robot_system.setGoal<PositionControllerDroneConnector, PositionYaw>(goal);
  }
};

template <class LogicStateMachineT>
struct PositionControlAbortActionFunctor_
    : ActionFunctor<Abort, QuadRotorSystem, LogicStateMachineT> {
  void run(const Abort &, QuadRotorSystem &robot_system, LogicStateMachineT &) {
    robot_system.abortController(HardwareType::Quadrotor);
  }
};

template <class LogicStateMachineT>
struct PositionControlTransitionGuardFunctor_
    : GuardFunctor<PositionYaw, QuadRotorSystem, LogicStateMachineT> {
  bool guard(const PositionYaw &goal, QuadRotorSystem &robot_system,
             LogicStateMachineT &) {
    parsernode::common::quaddata data = robot_system.getQuadData();
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

template <class LogicStateMachineT>
class PositionControlInternalActionFunctor_
    : InternalActionFunctor<QuadRotorSystem, LogicStateMachineT> {
  virtual void run(const InternalTransitionEvent &,
                   QuadRotorSystem &robot_system,
                   LogicStateMachineT &logic_state_machine) {
    // Get current goal
    PositionYaw goal =
        robot_system.getGoal<PositionControllerDroneConnector, PositionYaw>();
    // Get current position, yaw
    parsernode::common::quaddata data = robot_system.getQuadData();
    geometry_msgs::Vector3 current_position = data.localpos;
    double yaw = data.rpydata.z;
    // Define tolerance and check if reached goal
    // TODO Use a parameter for setting position tolerance
    double tolerance_pos = 1.0; // m
    double tolerance_yaw = 0.1; // rad
    if (std::abs(current_position.x - goal.x) < tolerance_pos &&
        std::abs(current_position.y - goal.y) < tolerance_pos &&
        std::abs(current_position.z - goal.z) < tolerance_pos &&
        std::abs(yaw - goal.yaw) < tolerance_yaw) {
      // Reached goal
      logic_state_machine.process_event(Completed());
    }
  }
};

template <class LogicStateMachineT>
using ReachingGoal_ =
    BaseState<QuadRotorSystem, LogicStateMachineT,
              PositionControlInternalActionFunctor_<LogicStateMachineT>>;
