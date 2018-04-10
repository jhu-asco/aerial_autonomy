#pragma once
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/actions_guards/hovering_functors.h>
#include <aerial_autonomy/actions_guards/shorting_action_sequence.h>
#include <aerial_autonomy/common/math.h>
#include <aerial_autonomy/common/proto_utils.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <aerial_autonomy/robot_systems/uav_vision_system.h>
#include <aerial_autonomy/trackers/id_tracking_strategy.h>
#include <aerial_autonomy/types/completed_event.h>
#include <aerial_autonomy/types/object_id.h>
#include <aerial_autonomy/types/reset_event.h>
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
    // \todo (Matt) Perhaps get goal based on a name instead of an index?
    auto goal =
        this->state_machine_config_.visual_servoing_state_machine_config()
            .relative_pose_goals()
            .Get(GoalIndex);
    robot_system.setGoal<RPYTRelativePoseVisualServoingConnector, PositionYaw>(
        conversions::protoPositionYawToPositionYaw(goal));
  }
};

/**
 * @brief Check the goal index commanded is available in the stored goal
 * arrays
 *
 * @tparam LogicStateMachineT Logic state machine used to process events
 * @tparam GoalIndex Waypoint index being commanded
 */
template <class LogicStateMachineT, int GoalIndex>
struct CheckGoalIndex_
    : EventAgnosticGuardFunctor<UAVVisionSystem, LogicStateMachineT> {
  bool guard(UAVVisionSystem &robot_system) {
    int goals_size =
        this->state_machine_config_.visual_servoing_state_machine_config()
            .relative_pose_goals()
            .size();
    if (GoalIndex >= goals_size) {
      return false;
    }
    return true;
  }
};

/**
* @brief Guard for initializing relative pose visual servoing for specific
* tracking id
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT, uint32_t MarkerId>
struct ExplicitIdVisualServoingGuardFunctor_
    : EventAgnosticGuardFunctor<UAVVisionSystem, LogicStateMachineT> {
  bool guard(UAVVisionSystem &robot_system) {
    robot_system.setTrackingStrategy(
        std::unique_ptr<TrackingStrategy>(new IdTrackingStrategy(MarkerId)));
    Position tracking_vector;
    if (!robot_system.getTrackingVector(tracking_vector)) {
      LOG(WARNING) << "Cannot track Marker Id since id not available: "
                   << MarkerId;
      return false;
    }
    return true;
  }
};

/**
* @brief Guard for initializing relative pose visual servoing for specific
* tracking id based on event
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct EventIdVisualServoingGuardFunctor_
    : public GuardFunctor<ObjectId, UAVVisionSystem, LogicStateMachineT> {
  bool guard(ObjectId const &event, UAVVisionSystem &robot_system) {
    auto pick_place_config =
        this->state_machine_config_.visual_servoing_state_machine_config()
            .pick_place_state_machine_config();
    for (auto place_group : pick_place_config.place_groups()) {
      if (proto_utils::contains(place_group.object_ids(), event.id)) {
        robot_system.setTrackingStrategy(std::unique_ptr<TrackingStrategy>(
            new IdTrackingStrategy(place_group.destination_id())));
        Position tracking_vector;
        if (!robot_system.getTrackingVector(tracking_vector)) {
          LOG(WARNING) << "Cannot track Marker Id since id not available: "
                       << place_group.destination_id();
          return false;
        }
        VLOG(2) << "Setting tracking id to " << place_group.destination_id();
        return true;
      }
    }
    LOG(WARNING) << "Could not find ID " << event.id
                 << " in configured place groups";
    return false;
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
    robot_system.setGoal<RPYTBasedPositionControllerDroneConnector,
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
* @tparam AbortEventT Event to send when controller is critical
*/
template <class LogicStateMachineT, class AbortEventT>
using RelativePoseVisualServoingInternalActionFunctor_ =
    boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
        UAVStatusInternalActionFunctor_<LogicStateMachineT>,
        ControllerStatusInternalActionFunctor_<
            LogicStateMachineT, RPYTRelativePoseVisualServoingConnector, true,
            AbortEventT>>>;

/**
* @brief Check tracking is valid
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT, class TrackingStrategy>
struct InitializeTrackerGuardFunctor_
    : EventAgnosticGuardFunctor<UAVVisionSystem, LogicStateMachineT> {
  bool guard(UAVVisionSystem &robot_system) {
    robot_system.setTrackingStrategy(
        std::unique_ptr<TrackingStrategy>(new TrackingStrategy()));
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
* @brief State that uses visual servoing to reach a desired goal.
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using VisualServoing_ =
    BaseState<UAVVisionSystem, LogicStateMachineT,
              VisualServoingInternalActionFunctor_<LogicStateMachineT>>;

/**
* @brief State that uses relative pose control functor to reach a desired goal.
*
* @tparam LogicStateMachineT Logic state machine used to process events
* @tparam AbortEventT Event to send when controller is critical
*/
template <class LogicStateMachineT, class AbortEventT>
using RelativePoseVisualServoing_ =
    BaseState<UAVVisionSystem, LogicStateMachineT,
              RelativePoseVisualServoingInternalActionFunctor_<
                  LogicStateMachineT, AbortEventT>>;
