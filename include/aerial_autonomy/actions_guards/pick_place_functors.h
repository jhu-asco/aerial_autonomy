#pragma once
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/actions_guards/hovering_functors.h>
#include <aerial_autonomy/actions_guards/manual_control_functors.h>
#include <aerial_autonomy/actions_guards/shorting_action_sequence.h>
#include <aerial_autonomy/actions_guards/visual_servoing_functors.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <aerial_autonomy/robot_systems/uav_arm_system.h>
#include <aerial_autonomy/types/completed_event.h>
#include <aerial_autonomy/types/reset_event.h>
#include <chrono>
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
  bool guard(UAVArmSystem &robot_system) { return true; }
};

template <class LogicStateMachineT, class StateT>
struct GrippingInternalActionFunctor_
    : public StateDependentInternalActionFunctor<UAVArmSystem,
                                                 LogicStateMachineT, StateT> {
  bool run(UAVArmSystem &robot_system, LogicStateMachineT &logic_state_machine,
           StateT &state) {
    VLOG(1) << "Gripping Object";
    if (state.monitorGrip(robot_system.grip(true))) {
      VLOG(1) << "Done Gripping!";
      logic_state_machine.process_event(Completed());
      return false;
    } else if (state.timeInState() > robot_system.gripTimeout()) {
      robot_system.resetGripper();
      LOG(WARNING) << "Timeout: Failed to grip!";
      logic_state_machine.process_event(Reset());
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

/**
* @brief State that monitors gripping status
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
class Gripping_
    : public BaseState<UAVArmSystem, LogicStateMachineT,
                       GrippingInternalActionFunctor_<
                           LogicStateMachineT, Gripping_<LogicStateMachineT>>> {
public:
  /**
  * @brief Function called when entering the state.  Logs time of entry.
  * @param e Event which triggered entry
  * @param fsm State's state machine
  * @tparam Event Type of event which triggered entry
  * @tparam FSM State machine type
  */
  template <class Event, class FSM> void on_entry(Event const &e, FSM &fsm) {
    // log start time
    entry_time_ = std::chrono::high_resolution_clock::now();
  }

  /**
  * @brief Check if grip has been successful for the required duration
  * @param Whether grip is currently successful
  * @return True if grip is successful for the duration, false otherwise
  */
  bool monitorGrip(bool grip_success) {
    if (grip_success) {
      if (!gripping_) {
        // start grip timer
        gripping_ = true;
        grip_start_time_ = std::chrono::high_resolution_clock::now();
      } else {
        // check grip timer
        return std::chrono::high_resolution_clock::now() - grip_start_time_ >
               required_grip_duration_;
      }
    } else {
      if (gripping_) {
        // stop grip timer
        gripping_ = false;
      }
    }
    return false;
  }
  /**
  * @brief Get the amount of time spent in the state
  * @return The amount of time spent in the state
  */
  std::chrono::duration<double> timeInState() {
    return std::chrono::duration<double>(
        std::chrono::high_resolution_clock::now() - entry_time_);
  }

private:
  std::chrono::time_point<std::chrono::high_resolution_clock> grip_start_time_;
  std::chrono::time_point<std::chrono::high_resolution_clock> entry_time_;
  std::chrono::milliseconds required_grip_duration_ =
      std::chrono::milliseconds(2000);
  bool gripping_ = false;
};
