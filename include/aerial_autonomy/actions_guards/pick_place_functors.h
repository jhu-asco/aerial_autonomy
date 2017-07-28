#pragma once
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/actions_guards/hovering_functors.h>
#include <aerial_autonomy/actions_guards/manual_control_functors.h>
#include <aerial_autonomy/actions_guards/shorting_action_sequence.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <aerial_autonomy/robot_systems/uav_arm_system.h>
#include <aerial_autonomy/types/completed_event.h>
#include <glog/logging.h>
#include <thread>

/**
* @brief Logic to grab an object, sleep for few seconds
* Abort UAV, Arm controllers
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct PickGuard_
    : EventAgnosticGuardFunctor<UAVArmSystem, LogicStateMachineT> {
  bool guard(UAVArmSystem &robot_system,
             LogicStateMachineT &logic_state_machine) {
    VLOG(1) << "Grip Object";
    robot_system.grip(true);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(robot_system.gripTimeout()));
    if (!robot_system.getCommandStatus()) {
      LOG(WARNING) << "Failed to grip object by timeout!";
      robot_system.grip(false);
      return false;
    }
    VLOG(1) << "Done Gripping!";
    return true;
  }
};

/**
* @brief Logic to check while reaching a visual servoing goal
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using PickInternalActionFunctor_ =
    boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
        UAVStatusInternalActionFunctor_<LogicStateMachineT>,
        ArmStatusInternalActionFunctor_<LogicStateMachineT>,
        ControllerStatusInternalActionFunctor_<
            LogicStateMachineT, VisualServoingControllerDroneConnector>,
        ControllerStatusInternalActionFunctor_<
            LogicStateMachineT, VisualServoingControllerArmConnector>>>;
/**
* @brief Logic to check arm power and manual mode
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using ManualControlArmInternalActionFunctor_ =
    boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
        ArmStatusInternalActionFunctor_<LogicStateMachineT>,
        ManualControlInternalActionFunctor_<LogicStateMachineT>>>;

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

template <class LogicStateMachineT>
struct VisualServoingArmTransitionActionFunctor_
    : EventAgnosticActionFunctor<UAVArmSystem, LogicStateMachineT> {
  void run(UAVArmSystem &robot_system,
           LogicStateMachineT &logic_state_machine) {
    VLOG(1) << "Setting Goal for visual servoing arm connector!";
    robot_system.setGoal<VisualServoingControllerArmConnector, tf::Transform>(
        robot_system.armGoalTransform());
    // Also ensure the gripper is open before going to pick objects
    robot_system.grip(false);
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
