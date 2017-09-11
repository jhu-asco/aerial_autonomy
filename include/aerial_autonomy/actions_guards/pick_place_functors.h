#pragma once
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/actions_guards/hovering_functors.h>
#include <aerial_autonomy/actions_guards/manual_control_functors.h>
#include <aerial_autonomy/actions_guards/shorting_action_sequence.h>
#include <aerial_autonomy/actions_guards/visual_servoing_functors.h>
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
  bool guard(UAVArmSystem &robot_system) {
    VLOG(1) << "Grip Object";
    bool success = robot_system.grip(true);
    // \todo this wait logic should be done on the arm plugin side and we should
    // be checking a status asynchronously for grip completion
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    if (!success) {
      LOG(WARNING) << "Failed to send grip command!";
      robot_system.resetGripper();
      return false;
    }
    VLOG(1) << "Done Gripping!";
    return true;
  }
};

template <class LogicStateMachineT>
struct PrePickGuard_
    : EventAgnosticGuardFunctor<UAVArmSystem, LogicStateMachineT> {
  bool guard(UAVArmSystem &robot_system) {
    if (!(bool(robot_system.getStatus<
               RelativePoseVisualServoingControllerDroneConnector>()) &&
          bool(robot_system
                   .getStatus<VisualServoingControllerArmConnector>()))) {
      LOG(WARNING) << "Both controllers not converged!";
      return false;
    }
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
            LogicStateMachineT, VisualServoingControllerArmConnector>,
        ControllerStatusInternalActionFunctor_<
            LogicStateMachineT,
            RelativePoseVisualServoingControllerDroneConnector, false>>>;
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
  bool guard(UAVArmSystem &robot_system_) {
    Position tracking_vector;
    return (robot_system_.getTrackingVector(tracking_vector) &&
            robot_system_.enabled());
  }
};

/**
 * @brief Set arm goal and set grip to false to start with.
 *
 * @tparam LogicStateMachineT State machine that contains the functor
 */
template <class LogicStateMachineT, int TransformIndex>
struct VisualServoingArmTransitionActionFunctor_
    : EventAgnosticActionFunctor<UAVArmSystem, LogicStateMachineT> {
  void run(UAVArmSystem &robot_system) {
    VLOG(1) << "Setting Goal for visual servoing arm connector!";
    robot_system.setGoal<VisualServoingControllerArmConnector, tf::Transform>(
        robot_system.armGoalTransform(TransformIndex));
    // Also ensure the gripper is in the right state to grip objects
    robot_system.resetGripper();
  }
};

/**
* @brief State that uses position control functor to reach a desired goal.
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using PrePickState_ = BaseState<UAVArmSystem, LogicStateMachineT,
                                PickInternalActionFunctor_<LogicStateMachineT>>;

/**
* @brief State that uses position control functor to reach a desired goal prior
* to picking.
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
class PickState_ : public RelativePoseVisualServoing_<LogicStateMachineT> {};
