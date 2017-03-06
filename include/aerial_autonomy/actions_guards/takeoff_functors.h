#pragma once
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <aerial_autonomy/robot_systems/uav_system.h>
#include <aerial_autonomy/basic_events.h>
#include <aerial_autonomy/types/completed_event.h>
#include <parsernode/common.h>

using namespace basic_events;

/**
* @brief action to take when starting takeoff
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct TakeoffTransitionActionFunctor_
    : EventAgnosticActionFunctor<UAVSystem, LogicStateMachineT> {
  void run(UAVSystem &robot_system, LogicStateMachineT &) {
    robot_system.takeOff();
  }
};

/**
* @brief action to perform when aborting takeoff
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct TakeoffAbortActionFunctor_
    : EventAgnosticActionFunctor<UAVSystem, LogicStateMachineT> {
  void run(UAVSystem &robot_system, LogicStateMachineT &) {
    robot_system.land();
  }
};

/**
* @brief Guard to avoid accidental takeoff
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct TakeoffTransitionGuardFunctor_
    : EventAgnosticGuardFunctor<UAVSystem, LogicStateMachineT> {
  bool guard(UAVSystem &robot_system, LogicStateMachineT &) {
    parsernode::common::quaddata data = robot_system.getUAVData();
    bool result = false;
    // Check for voltage
    // TODO Set a parameter for minimum battery percent
    if (data.batterypercent > 40) {
      result = true;
    } else {
      std::cout << "Battery too low! " << data.batterypercent
                << "% Cannot takeoff" << std::endl;
    }
    return result;
  }
};

/**
* @brief Check when takeoff is complete, and ensure enough battery voltage
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct TakeoffInternalActionFunctor_
    : EventAgnosticActionFunctor<UAVSystem, LogicStateMachineT> {
  /**
  * @brief Function to check when takeoff is complete.
  * If battery is low while takeoff, trigger Land event
  *
  *  \todo add a parameter for height when to transition from takeoff to hovering
  *
  * @param robot_system robot system to get sensor data
  * @param logic_state_machine logic state machine to trigger events
  */
  void run(UAVSystem &robot_system, LogicStateMachineT &logic_state_machine) {
    parsernode::common::quaddata data = robot_system.getUAVData();
    // If battery too low abort and goto landed mode
    if (data.batterypercent < 40) {
      logic_state_machine.process_event(Land());
    }
    // Transition to hovering state once reached high altitude
    // TODO add a parameter for transitioning from takeoff to hovering
    if (data.altitude > 1.0) {
      logic_state_machine.process_event(Completed());
    }
  }
};

/**
* @brief Taking off state that uses the internal action functor
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using TakingOff_ = BaseState<UAVSystem, LogicStateMachineT,
                             TakeoffInternalActionFunctor_<LogicStateMachineT>>;
