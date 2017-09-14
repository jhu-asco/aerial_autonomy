#pragma once
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/actions_guards/hovering_functors.h>
#include <aerial_autonomy/actions_guards/shorting_action_sequence.h>
#include <aerial_autonomy/common/math.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <aerial_autonomy/robot_systems/uav_vision_system.h>
#include <aerial_autonomy/types/completed_event.h>
#include <glog/logging.h>
#include <parsernode/common.h>
/**
* @brief Action for initializing relative pose visual servoing
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct ResetRelativePoseVisualServoingTransitionActionFunctor_
    : EventAgnosticActionFunctor<UAVVisionSystem, LogicStateMachineT> {
  void run(UAVVisionSystem &robot_system) {
    robot_system.resetRelativePoseController();
  }
};
/**
* @brief Action for initializing relative pose visual servoing
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT, int GoalIndex, bool SetHome = true>
struct RelativePoseVisualServoingTransitionActionFunctor_
    : EventAgnosticActionFunctor<UAVVisionSystem, LogicStateMachineT> {
  void run(UAVVisionSystem &robot_system) {
    if (SetHome) {
      VLOG(1) << "Selecting home location";
      robot_system.setHomeLocation();
    }
    VLOG(1) << "Setting goal " << GoalIndex
            << " for relative pose visual servoing drone connector!";
    robot_system.setGoal<RelativePoseVisualServoingControllerDroneConnector,
                         PositionYaw>(robot_system.relativePoseGoal(GoalIndex));
  }
};

/**
* @brief Action for initializing relative pose visual servoing for specific
* tracking id
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT, uint32_t MarkerId>
struct ExplicitIdVisualServoingGuardFunctor_
    : EventAgnosticGuardFunctor<UAVVisionSystem, LogicStateMachineT> {
  bool guard(UAVVisionSystem &robot_system) {
    robot_system.setExplicitIdVisualServoing(MarkerId);
    Position tracking_vector;
    if (!robot_system.getTrackingVector(tracking_vector)) {
      LOG(WARNING) << "Cannot track Marker Id since id not available: "
                   << MarkerId;
      robot_system.resetExplicitIdVisualServoing();
      return false;
    }
    return true;
  }
};

/**
* @brief Empty for now
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct VisualServoingTransitionActionFunctor_
    : EventAgnosticActionFunctor<UAVVisionSystem, LogicStateMachineT> {
  void run(UAVVisionSystem &robot_system) {}
};

/**
* @brief
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct VisualServoingAbortActionFunctor_
    : EventAgnosticActionFunctor<UAVVisionSystem, LogicStateMachineT> {
  void run(UAVVisionSystem &robot_system) {
    LOG(WARNING) << "Aborting visual servoing controller";
    robot_system.abortController(HardwareType::UAV);
  }
};

/**
* @brief Action to reach a pre designated point
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct GoHomeTransitionActionFunctor_
    : EventAgnosticActionFunctor<UAVVisionSystem, LogicStateMachineT> {
  void run(UAVVisionSystem &robot_system) {
    PositionYaw home_location = robot_system.getHomeLocation();
    VLOG(1) << "Going home";
    robot_system.setGoal<VelocityBasedPositionControllerDroneConnector,
                         PositionYaw>(home_location);
  }
};

/**
* @brief Guard for home transition
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct GoHomeTransitionGuardFunctor_
    : EventAgnosticGuardFunctor<UAVVisionSystem, LogicStateMachineT> {
  bool guard(UAVVisionSystem &robot_system) {
    return robot_system.isHomeLocationSpecified();
  }
};

/**
* @brief Logic to check while reaching a visual servoing goal
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using VisualServoingInternalActionFunctor_ =
    boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
        UAVStatusInternalActionFunctor_<LogicStateMachineT>,
        ControllerStatusInternalActionFunctor_<
            LogicStateMachineT, VisualServoingControllerDroneConnector>>>;

/**
* @brief Logic to check while reaching a visual servoing goal
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using RelativePoseVisualServoingInternalActionFunctor_ =
    boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
        UAVStatusInternalActionFunctor_<LogicStateMachineT>,
        ControllerStatusInternalActionFunctor_<
            LogicStateMachineT,
            RelativePoseVisualServoingControllerDroneConnector>>>;

/**
* @brief Check tracking is valid
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct InitializeTrackerGuardFunctor_
    : EventAgnosticGuardFunctor<UAVVisionSystem, LogicStateMachineT> {
  bool guard(UAVVisionSystem &robot_system) {
    if (!robot_system.initializeTracker()) {
      LOG(WARNING) << "Could not initialize tracking.";
      return false;
    }
    Position tracking_vector;
    if (!robot_system.getTrackingVector(tracking_vector)) {
      LOG(WARNING) << "Lost tracking while servoing.";
      return false;
    }
    return true;
  }
};

/**
* @brief Check tracking is valid before starting visual servoing
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct VisualServoingTransitionGuardFunctor_
    : EventAgnosticGuardFunctor<UAVVisionSystem, LogicStateMachineT> {
  bool guard(UAVVisionSystem &robot_system) {
    Position tracking_vector;
    if (!robot_system.getTrackingVector(tracking_vector)) {
      LOG(WARNING) << "Lost tracking while servoing.";
      return false;
    }
    VLOG(1) << "Setting tracking vector";
    double desired_distance = robot_system.getConfiguration()
                                  .uav_vision_system_config()
                                  .desired_visual_servoing_distance();
    double tracking_vector_norm = tracking_vector.norm();
    if (tracking_vector_norm < 1e-6) {
      LOG(WARNING) << "Tracking vector too small cannot initialize direction";
      return false;
    } else {
      // \todo Matt: could possibly move this block to the action functor
      VLOG(1) << "Selecting home location";
      robot_system.setHomeLocation();
      robot_system.setGoal<VisualServoingControllerDroneConnector, Position>(
          tracking_vector * desired_distance / tracking_vector_norm);
    }
    return true;
  }
};

/**
* @brief State that uses position control functor to reach a desired goal.
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using VisualServoing_ =
    BaseState<UAVVisionSystem, LogicStateMachineT,
              VisualServoingInternalActionFunctor_<LogicStateMachineT>>;

/**
* @brief State that uses relative pose control functor to reach a desired goal.
* Also resets any explicit ids set by transition action functor when exiting the
* state
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct RelativePoseVisualServoing_
    : BaseState<UAVVisionSystem, LogicStateMachineT,
                RelativePoseVisualServoingInternalActionFunctor_<
                    LogicStateMachineT>> {
  template <class Event, class FSM>
  void on_exit(Event const &, FSM &logic_state_machine) {
    UAVVisionSystem &robot_system = this->getRobotSystem(logic_state_machine);
    VLOG(1) << "Resetting any explicitly set marker ids";
    robot_system.resetExplicitIdVisualServoing();
  }
};
