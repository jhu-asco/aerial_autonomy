#pragma once
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/actions_guards/hovering_functors.h>
#include <aerial_autonomy/actions_guards/manual_control_functors.h>
#include <aerial_autonomy/actions_guards/shorting_action_sequence.h>
#include <aerial_autonomy/actions_guards/visual_servoing_functors.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <aerial_autonomy/logic_states/timed_state.h>
#include <aerial_autonomy/robot_systems/uav_arm_system.h>
#include <aerial_autonomy/types/completed_event.h>
#include <aerial_autonomy/types/reset_event.h>
#include <chrono>
#include <glog/logging.h>
#include <thread>

// Forward declaration for GrippingInternalActionFunctor_
template <class LogicStateMachineT> class PickState_;

/**
* @brief Checks whether grip command has completed or timed out
* @tparam LogicStateMachineT Logic state machine used to process events
* @tparam StateT State which stores gripper timer state
*/
template <class LogicStateMachineT>
struct GrippingInternalActionFunctor_
    : public StateDependentInternalActionFunctor<
          UAVArmSystem, LogicStateMachineT, PickState_<LogicStateMachineT>> {
  bool run(UAVArmSystem &robot_system, LogicStateMachineT &logic_state_machine,
           PickState_<LogicStateMachineT> &state) {
    VLOG(1) << "Gripping Object";
    if (state.monitorGrip(robot_system.grip(true))) {
      VLOG(1) << "Done Gripping!";
      logic_state_machine.process_event(Completed());
      return false;
    } else if (state.timeInState() > robot_system.gripTimeout()) {
      // \todo Matt Put this in its own action functor.  The timeout should be
      // based on a state config, not a robot_system config
      robot_system.resetGripper();
      LOG(WARNING) << "Timeout: Failed to grip!";
      logic_state_machine.process_event(Reset());
      return false;
    }
    return true;
  }
};

/**
* @brief Logic to check while reaching a visual servoing and arm end effector
* goal
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using PrePickInternalActionFunctor_ =
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
 * @tparam TransformIndex Index of goal transform
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
* @brief Action to reach a pre designated point A
*
* @tparam LogicStateMachineT Logic state machine used to process events
* @tparam Index Which waypoint we are reaching to
*/
template <class LogicStateMachineT, int Index>
struct GoToWayPointTransitionActionFunctor_
    : EventAgnosticActionFunctor<UAVArmSystem, LogicStateMachineT> {
  void run(UAVArmSystem &robot_system) {
    PositionYaw way_point = robot_system.getWayPoint(Index);
    VLOG(1) << "Going to waypoint " << Index;
    robot_system.setGoal<VelocityBasedPositionControllerDroneConnector,
                         PositionYaw>(way_point);
  }
};

/**
* @brief Guard for waypoint A transition
*
* @tparam LogicStateMachineT Logic state machine used to process events
* @tparam Index which waypoint is it guarding
*/
template <class LogicStateMachineT, int Index>
struct GoToWayPointTransitionGuardFunctor_
    : EventAgnosticGuardFunctor<UAVArmSystem, LogicStateMachineT> {
  bool guard(UAVArmSystem &robot_system) {
    bool result = robot_system.checkWayPointIndex(Index);
    if (!result) {
      LOG(WARNING) << "Index: " << Index
                   << " not available in waypoint vector in config file";
    }
    return result;
  }
};

/**
* @brief State that uses position control functor to reach a desired goal.
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using PrePickState_ =
    BaseState<UAVArmSystem, LogicStateMachineT,
              PrePickInternalActionFunctor_<LogicStateMachineT>>;

/**
* @brief State that uses position control functor to reach a desired goal for
* picking and monitors the gripper status
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
class PickState_
    : public TimedState<
          UAVArmSystem, LogicStateMachineT,
          boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
              UAVStatusInternalActionFunctor_<LogicStateMachineT>,
              ArmStatusInternalActionFunctor_<LogicStateMachineT>,
              ControllerStatusInternalActionFunctor_<
                  LogicStateMachineT,
                  RelativePoseVisualServoingControllerDroneConnector, false>,
              GrippingInternalActionFunctor_<LogicStateMachineT>>>> {
public:
  /**
  * @brief Check if grip has been successful for the required duration
  * @param Whether grip is currently successful
  * @return True if grip is successful for the duration, false otherwise
  */
  bool monitorGrip(bool grip_success) {
    bool grip_duration_success = false;
    if (grip_success) {
      if (!gripping_) {
        // start grip timer
        gripping_ = true;
        grip_start_time_ = std::chrono::high_resolution_clock::now();
      } else {
        // check grip timer
        grip_duration_success =
            std::chrono::high_resolution_clock::now() - grip_start_time_ >
            required_grip_duration_;
      }
    } else {
      if (gripping_) {
        // stop grip timer
        gripping_ = false;
      }
    }
    return grip_duration_success;
  }

private:
  std::chrono::time_point<std::chrono::high_resolution_clock> grip_start_time_;
  std::chrono::milliseconds required_grip_duration_ =
      std::chrono::milliseconds(1000);
  bool gripping_ = false;
};
