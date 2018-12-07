#pragma once
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/actions_guards/hovering_functors.h>
#include <aerial_autonomy/actions_guards/shorting_action_sequence.h>
#include <aerial_autonomy/common/math.h>
#include <aerial_autonomy/common/proto_utils.h>
#include <aerial_autonomy/controller_connectors/relative_pose_visual_servoing_controller_drone_connector.h>
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
    auto state_machine_config =
        this->state_machine_config_.visual_servoing_state_machine_config();
    if (GoalIndex >= state_machine_config.relative_pose_goals().size()) {
      LOG(ERROR) << "GoalIndex: " << GoalIndex << " Relative Pose goals size: "
                 << state_machine_config.relative_pose_goals().size();
    }
    auto goal = state_machine_config.relative_pose_goals().Get(GoalIndex);
    PositionYaw position_yaw_goal =
        conversions::protoPositionYawToPositionYaw(goal);
    auto connector_type = state_machine_config.connector_type();
    switch (connector_type) {
    case VisualServoingStateMachineConfig::RPYTPose:
      robot_system.setGoal<RPYTRelativePoseVisualServoingConnector,
                           PositionYaw>(position_yaw_goal);
      break;
    case VisualServoingStateMachineConfig::RPYTRef:
      robot_system.setGoal<
          UAVVisionSystem::RPYTVisualServoingReferenceConnectorT, PositionYaw>(
          position_yaw_goal);
      break;
    case VisualServoingStateMachineConfig::MPC:
      robot_system.setGoal<
          UAVVisionSystem::MPCVisualServoingReferenceConnectorT, PositionYaw>(
          position_yaw_goal);
      break;
    case VisualServoingStateMachineConfig::VelPose:
      robot_system.setGoal<RelativePoseVisualServoingControllerDroneConnector,
                           PositionYaw>(position_yaw_goal);
      break;
    case VisualServoingStateMachineConfig::HeadingDepth:
      // Maybe use relativepose visual servoing drone connector instead of
      // this??
      VisualServoingTransitionGuardFunctor_<LogicStateMachineT>().guard(
          robot_system);
      break;
    default:
      LOG(WARNING) << "Unknown visual servoing connector";
      break;
    }
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
template <class LogicStateMachineT, class AbortEventT>
struct VisualServoingStatus_
    : InternalActionFunctor<UAVSystem, LogicStateMachineT> {
  bool run(UAVSystem &robot_system, LogicStateMachineT &logic_state_machine) {
    // Check config for which connector to use
    auto connector_type = logic_state_machine.base_state_machine_config_
                              .visual_servoing_state_machine_config()
                              .connector_type();
    bool result = false;
    switch (connector_type) {
    case VisualServoingStateMachineConfig::RPYTPose:
      result = ControllerStatusInternalActionFunctor_<
                   LogicStateMachineT, RPYTRelativePoseVisualServoingConnector,
                   true, AbortEventT>()
                   .run(robot_system, logic_state_machine);
      break;
    case VisualServoingStateMachineConfig::RPYTRef:
      result =
          ControllerStatusInternalActionFunctor_<
              LogicStateMachineT,
              RPYTBasedReferenceConnector<Eigen::VectorXd, Eigen::VectorXd>,
              true, AbortEventT>()
              .run(robot_system, logic_state_machine);
      result &= ControllerStatusInternalActionFunctor_<
                    LogicStateMachineT,
                    UAVVisionSystem::RPYTVisualServoingReferenceConnectorT,
                    false, AbortEventT>()
                    .run(robot_system, logic_state_machine);
      break;
    case VisualServoingStateMachineConfig::MPC:
      result =
          ControllerStatusInternalActionFunctor_<LogicStateMachineT,
                                                 MPCControllerQuadConnector,
                                                 true, AbortEventT>()
              .run(robot_system, logic_state_machine);
      result &= ControllerStatusInternalActionFunctor_<
                    LogicStateMachineT,
                    UAVVisionSystem::MPCVisualServoingReferenceConnectorT,
                    false, AbortEventT>()
                    .run(robot_system, logic_state_machine);
      break;
    case VisualServoingStateMachineConfig::HeadingDepth:
      result = ControllerStatusInternalActionFunctor_<
                   LogicStateMachineT, VisualServoingControllerDroneConnector,
                   true, AbortEventT>()
                   .run(robot_system, logic_state_machine);
      break;
    case VisualServoingStateMachineConfig::VelPose:
      result = ControllerStatusInternalActionFunctor_<
                   LogicStateMachineT,
                   RelativePoseVisualServoingControllerDroneConnector, true,
                   AbortEventT>()
                   .run(robot_system, logic_state_machine);
      break;
    }
    return result;
  }
};

/**
* @brief Logic to check while reaching a visual servoing goal
*
* @tparam LogicStateMachineT Logic state machine used to process events
* @tparam AbortEventT Event to send when controller is critical
*/
template <class LogicStateMachineT, class AbortEventT>
using VisualServoingInternalActionFunctor_ =
    boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
        UAVStatusInternalActionFunctor_<LogicStateMachineT>,
        VisualServoingStatus_<LogicStateMachineT, AbortEventT>>>;

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
* @brief State that uses relative pose control functor to reach a desired goal.
*
* @tparam LogicStateMachineT Logic state machine used to process events
* @tparam AbortEventT Event to send when controller is critical
*/
template <class LogicStateMachineT, class AbortEventT>
using VisualServoing_ = BaseState<
    UAVVisionSystem, LogicStateMachineT,
    VisualServoingInternalActionFunctor_<LogicStateMachineT, AbortEventT>>;
