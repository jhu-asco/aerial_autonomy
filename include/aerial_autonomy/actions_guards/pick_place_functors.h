#pragma once
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/actions_guards/manual_control_functors.h>
#include <aerial_autonomy/actions_guards/uav_status_functor.h>
#include <aerial_autonomy/common/math.h>
#include <aerial_autonomy/logic_states/base_state.h>
//\todo replace system
//#include <aerial_autonomy/robot_systems/uav_arm_system.h>
#include <aerial_autonomy/types/completed_event.h>
#include <glog/logging.h>

/**
* @brief Logic to grab an object, sleep for few seconds
* Abort UAV, Arm controllers
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct PickAction_
    : EventAgnosticActionFunctor<UAVArmSystem, LogicStateMachineT> {
  void run(UAVArmSystem &robot_system,
           LogicStateMachineT &logic_state_machine) {
    VLOG(1) << "Aborting Controllers";
    robot_system.abortController(HardwareType::UAV);
    robot_system.abortController(HardwareType::Arm);
    ///\todo Add code to pick an object
    // Check type of event to decide what to do
    VLOG(1) << "Grip Object";
    robot_system.grip(true);
    // Timeout
    // Check gripping is done to print warning if not done
    // Exit irrespective of gripping done or not after timeout
  }
};

/**
* @brief Logic to check while reaching a visual servoing goal
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct PickInternalActionFunctor_
    : UAVStatusActionFunctor<UAVArmSystem, LogicStateMachineT> {
  /**
  * @brief check if quad and arm reached VS goal and
  * trigger completed event
  *
  * @param robot_system robot system to get sensor data
  * @param logic_state_machine logic state machine to trigger events
  */
  void statusIndependentRun(UAVArmSystem &robot_system,
                            LogicStateMachineT &logic_state_machine) {
    ControllerStatus status =
        robot_system.getStatus<VisualServoingControllerDroneConnector>();
    //\todo Check controller status of arm along with quad
    // Define tolerance and check if reached goal
    if (status == ControllerStatus::Completed) {
      VLOG(1) << "Reached goal";
      logic_state_machine.process_event(Completed());
    } else if (status == ControllerStatus::Critical) {
      LOG(WARNING) << "Lost tracking while servoing.";
      logic_state_machine.process_event(be::Abort());
    }
  }
};

/**
* @brief Logic to check arm power and manual mode
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/

template <class LogicStateMachineT>
struct ManualControlArmInternalActionFunctor_
    : EventAgnosticActionFunctor<UAVArmSystem, LogicStateMachineT> {
  /**
  * @brief The parent functor to switch to other states based on state of
  * rc switch
  */
  ManualControlInternalActionFunctor_<LogicStateMachineT> parent_functor_;
  /**
  * @brief Check if arm is enabled and print warning otherwise. Also call
  * parent functor for checking rc switch
  *
  * @param robot_system robot system to get sensor data
  * @param logic_state_machine logic state machine to trigger events
  */
  void run(UAVArmSystem &robot_system,
           LogicStateMachineT &logic_state_machine) {
    if (!robot_system.enabled()) {
      LOG(WARNING) << "Arm not enabled!";
      robot_system.power(true); // Try to enable arm
    } else {
      // Since arm is enabled we can switch to other states
      parent_functor_.run(robot_system, logic_state_machine);
    }
  }
};
/**
* @brief Check tracking is valid before starting visual servoing and arm is
* enabled before picking objects
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct PickTransitionGuardFunctor_
    : EventAgnosticGuardFunctor<UAVArmSystem, LogicStateMachineT> {
  bool guard(UAVArmSystem &robot_system_, LogicStateMachineT &) {
    Position tracking_vector;
    return (robot_system_.getTrackingVector(tracking_vector) &&
            robot_system_.enabled());
  }
};

/**
* @brief State that uses position control functor to reach a desired goal.
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using PickState_ = BaseState<UAVArmSystem, LogicStateMachineT,
                             PickInternalActionFunctor_<LogicStateMachineT>>;
/**
* @brief State that checks arm status along with regular manual control
* state
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using ManualControlArmState_ =
    BaseState<UAVArmSystem, LogicStateMachineT,
              ManualControlArmInternalActionFunctor_<LogicStateMachineT>>;
