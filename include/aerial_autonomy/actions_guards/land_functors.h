#pragma once
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <aerial_autonomy/robot_systems/uav_system.h>
#include <aerial_autonomy/types/completed_event.h>
#include <aerial_autonomy/types/manual_control_event.h>
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
    /// \todo Have to abort all hardware controllers not just UAV.
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
    // If hardware is not allowing us to control UAV
    if (!data.rc_sdk_control_switch) {
      VLOG(1) << "Switching to Manual UAV state";
      logic_state_machine.process_event(ManualControlEvent());
    } else if (data.localpos.z <
               robot_system.getConfiguration().landing_height()) {
      /// \todo (Gowtham) Can also use uav status here
      VLOG(1) << "Completed Landing";
      logic_state_machine.process_event(Completed());
    }
  }
};

/**
* @brief Internal action to switch to manual uav state
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct LandedInternalActionFunctor_
    : EventAgnosticActionFunctor<UAVSystem, LogicStateMachineT> {
  /**
  * @brief Internal function to switch to manual uav state
  *
  * @param robot_system robot system to get sensor data
  * @param logic_state_machine logic state machine to trigger events
  */
  void run(UAVSystem &robot_system, LogicStateMachineT &logic_state_machine) {
    parsernode::common::quaddata data = robot_system.getUAVData();
    // If hardware is not allowing us to control UAV
    if (!data.rc_sdk_control_switch) {
      VLOG(1) << "Switching to Manual UAV state";
      logic_state_machine.process_event(ManualControlEvent());
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
/**
* @brief State that uses internal action when already landed
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using Landed_ = BaseState<UAVSystem, LogicStateMachineT,
                          LandedInternalActionFunctor_<LogicStateMachineT>>;
