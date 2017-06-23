#pragma once
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <aerial_autonomy/robot_systems/arm_system.h>
#include <aerial_autonomy/types/completed_event.h>
#include <aerial_autonomy/uav_basic_events.h>
#include <glog/logging.h>
#include <parsernode/common.h>

namespace be = uav_basic_events;

/**
* @brief action to fold arm
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct ArmFoldTransitionActionFunctor_
    : EventAgnosticActionFunctor<ArmSystem, LogicStateMachineT> {
  void run(ArmSystem &robot_system, LogicStateMachineT &) {
    VLOG(1) << "Folding Arm!";
    robot_system.foldArm();
  }
};

/**
* @brief action to move arm to right angle
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct ArmRightFoldTransitionActionFunctor_
    : EventAgnosticActionFunctor<ArmSystem, LogicStateMachineT> {
  void run(ArmSystem &robot_system, LogicStateMachineT &) {
    VLOG(1) << "Folding Arm to right angle!";
    robot_system.rightArm();
  }
};

/**
* @brief Power off arm
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct ArmPoweroffTransitionActionFunctor_
    : EventAgnosticActionFunctor<ArmSystem, LogicStateMachineT> {
  void run(ArmSystem &robot_system, LogicStateMachineT &) {
    VLOG(1) << "Powering off Arm!";
    robot_system.power(false);
  }
};

/**
* @brief Power on the arm
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct ArmPoweronTransitionActionFunctor_
    : EventAgnosticActionFunctor<ArmSystem, LogicStateMachineT> {
  void run(ArmSystem &robot_system, LogicStateMachineT &) {
    VLOG(1) << "Powering on Arm!";
    robot_system.power(true);
  }
};

/**
* @brief Abort arm controller
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct AbortArmController_
    : EventAgnosticActionFunctor<ArmSystem, LogicStateMachineT> {
  void run(ArmSystem &robot_system, LogicStateMachineT &) {
    LOG(WARNING) << "Aborting arm controller";
    robot_system.abortController(HardwareType::Arm);
  }
};

/**
* @brief Check when folding arm is complete
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct ArmFoldInternalActionFunctor_
    : EventAgnosticActionFunctor<ArmSystem, LogicStateMachineT> {
  /**
  * @brief Function to check when folding arm is complete.
  * If arm is not enabled, then abort
  *
  * @param robot_system robot system to get sensor data
  * @param logic_state_machine logic state machine to trigger events
  */
  void run(ArmSystem &robot_system, LogicStateMachineT &logic_state_machine) {
    if (robot_system.getCommandStatus()) {
      VLOG(1) << "Completed Folding arm!";
      logic_state_machine.process_event(be::Takeoff());
    } else if (!robot_system.enabled()) {
      LOG(WARNING) << "Arm not enabled!";
      logic_state_machine.process_event(be::Abort());
    }
  }
};

/**
* @brief Taking off state that uses the internal action functor
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using ArmFolding_ =
    BaseState<ArmSystem, LogicStateMachineT,
              ArmFoldInternalActionFunctor_<LogicStateMachineT>>;
/**
* @brief Same state as above. Used to distinguish between landing and takeoff
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
class ArmPreTakeoffFolding_ : public ArmFolding_<LogicStateMachineT> {};
/**
* @brief Same state as above. Used to distinguish between landing and takeoff
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
class ArmPreLandingFolding_ : public ArmFolding_<LogicStateMachineT> {};
