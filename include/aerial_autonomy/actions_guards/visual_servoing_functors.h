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
    robot_system.setGoal<PositionControllerDroneConnector, PositionYaw>(
        home_location);
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
* @brief Check tracking is valid before starting visual servoing
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct VisualServoingTransitionGuardFunctor_
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
