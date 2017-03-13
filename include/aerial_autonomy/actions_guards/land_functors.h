#pragma once
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/basic_events.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <aerial_autonomy/robot_systems/uav_system.h>
#include <aerial_autonomy/types/completed_event.h>
#include <parsernode/common.h>

/**
* @brief Transition action when starting to land
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct LandTransitionActionFunctor_
    : EventAgnosticActionFunctor<UAVSystem, LogicStateMachineT> {
  void run(UAVSystem &robot_system, LogicStateMachineT &) {
    // TODO Have to abort all hardware controllers not just UAV.
    // Need to iterate through enum class HardwareType
    VLOG(1) << "Aborting UAV Controllers";
    robot_system.abortController(HardwareType::UAV);
    VLOG(1) << "Landing";
    robot_system.land();
  }
};

/**
* @brief Internal action to figure out when landing is complete
*
* \todo (Gowtham) How to abort Land??
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct LandInternalActionFunctor_
    : EventAgnosticActionFunctor<UAVSystem, LogicStateMachineT> {
  /**
  * @brief Internal function to check when landing is complete
  *
  * @param robot_system robot system to get sensor data
  * @param logic_state_machine logic state machine to trigger events
  */
  void run(UAVSystem &robot_system, LogicStateMachineT &logic_state_machine) {
    parsernode::common::quaddata data = robot_system.getUAVData();
    // Can also use uav status here TODO (Gowtham)
    if (data.altitude < robot_system.getConfiguration().landing_height()) {
      VLOG(1) << "Completed Landing";
      logic_state_machine.process_event(Completed());
    }
  }
};

/**
* @brief State that uses internal action for landing
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using Landing_ = BaseState<UAVSystem, LogicStateMachineT,
                           LandInternalActionFunctor_<LogicStateMachineT>>;
