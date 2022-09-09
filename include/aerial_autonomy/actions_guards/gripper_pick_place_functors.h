#pragma once
#include "grip_config.pb.h"
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/actions_guards/hovering_functors.h>
#include <aerial_autonomy/actions_guards/manual_control_functors.h>
#include <aerial_autonomy/actions_guards/shorting_action_sequence.h>
#include <aerial_autonomy/actions_guards/visual_servoing_functors.h>
#include <aerial_autonomy/actions_guards/pick_place_functors.h>
#include <aerial_autonomy/common/conversions.h>
#include <aerial_autonomy/common/proto_utils.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <aerial_autonomy/logic_states/timed_state.h>
#include <aerial_autonomy/pick_place_events.h>
#include <aerial_autonomy/robot_systems/uav_arm_system.h>
#include <aerial_autonomy/types/completed_event.h>
#include <aerial_autonomy/types/object_id.h>
#include <aerial_autonomy/types/polynomial_reference_trajectory.h>
#include <aerial_autonomy/types/reset_event.h>
#include <chrono>
#include <glog/logging.h>
#include <thread>

// Forward declaration for GripInternalActionFunctor_
template <class LogicStateMachineT> class GripState_;

// Forward declaration for ReachingWaypointInternalActionFunctorWithObject_
template <class LogicStateMachineT, int StartIndex, int EndIndex, bool CheckForGoal>
struct FollowingWaypointSequenceWithObject_;

// // Forward declaration for WaitingForPickInternalActionFunctor_
// template <class LogicStateMachineT> struct WaitingForPick_;

/**
* @brief Transition action to perform when going into position controlled hover
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct HoverPositionControlTransitionActionFunctor_
    : EventAgnosticActionFunctor<UAVSystem, LogicStateMachineT> {
  void run(UAVSystem &robot_system) {
    tf::StampedTransform start_pose = robot_system.getPose();
    auto &reference_config =
        this->state_machine_config_.poly_reference_config();
    PositionYaw start_position_yaw;
    conversions::tfToPositionYaw(start_position_yaw, start_pose);
    ReferenceTrajectoryPtr<Eigen::VectorXd, Eigen::VectorXd> reference(
        new PolynomialReferenceTrajectory(start_position_yaw, start_position_yaw,
                                          reference_config));
    robot_system
        .setGoal<RPYTBasedReferenceConnector<Eigen::VectorXd, Eigen::VectorXd>,
                 ReferenceTrajectoryPtr<Eigen::VectorXd, Eigen::VectorXd>>(
            reference);
  }
};

/**
* @brief action to grip for specific object ID
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct ArmGripIDActionFunctor_
    : ActionFunctor<ObjectId, UAVArmSystem, LogicStateMachineT> {
  void run(ObjectId const &event, UAVArmSystem &robot_system) {
    auto pick_place_config =
        this->state_machine_config_.visual_servoing_state_machine_config()
            .pick_place_state_machine_config();
    // Find the corresponding place group
    for (auto place_group : pick_place_config.place_groups()) {
      if (proto_utils::contains(place_group.object_ids(), event.id)) {
        // Set grip angle
        std::vector<double> joint_angles;
        joint_angles.push_back(place_group.gripper_angle());
        VLOG(1) << "Gripping! With angle: " << joint_angles[0];
        robot_system.setJointAngles(joint_angles);
      }
    }
  }
};

/**
* @brief Checks whether grip command has completed or timed out
* @tparam LogicStateMachineT Logic state machine used to process events
* @tparam StateT State which stores gripper timer state
*/
template <class LogicStateMachineT>
struct GripInternalActionFunctor_
    : public StateDependentInternalActionFunctor<
          UAVArmSystem, LogicStateMachineT, GripState_<LogicStateMachineT>> {
  bool run(UAVArmSystem &robot_system, LogicStateMachineT &logic_state_machine,
           GripState_<LogicStateMachineT> &state) {
    bool has_grip = robot_system.gripStatus();
    if (state.monitorGrip(has_grip)) {
      VLOG(1) << "Done Gripping!";
      uint32_t tracked_id;
      if (robot_system.getTrackingVectorId(tracked_id)) {
        ObjectId id_event(tracked_id);
        VLOG(1) << "Picked object with ID " << tracked_id;
        logic_state_machine.process_event(id_event);
      } else {
        LOG(WARNING) << "Could not retrieve object ID!";
        logic_state_machine.process_event(Reset());
      }
      return false;
    } else if (state.timeInState() > state.gripTimeout()) {
      robot_system.resetGripper();
      LOG(WARNING) << "Timeout: Failed to grip!";
      logic_state_machine.process_event(Reset());
      return false;
    }
    return true;
  }
};

/**
* @brief Checks whether grip has been maintained 
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct GripMaintainInternalActionFunctor_
    : InternalActionFunctor<UAVArmSystem, LogicStateMachineT> {
  /**
  * @brief Checks if robot reports that it has the grip
  *
  * @param robot_system robot system to get sensor data
  * @param logic_state_machine logic state machine to trigger events
  */
  bool run(UAVArmSystem &robot_system, LogicStateMachineT &logic_state_machine) {
    bool has_grip = robot_system.gripStatus();
    if (!has_grip) {
      LOG(WARNING) << "Grip not maintained!";
      logic_state_machine.process_event(Reset());
    }
    return true;
  }
};

// /**
// * @brief Check tracking is valid before starting visual servoing and arm is
// * enabled before picking objects
// *
// * @tparam LogicStateMachineT Logic state machine used to process events
// */
// template <class LogicStateMachineT>
// struct ArmTrackingGuardFunctor_
//     : EventAgnosticGuardFunctor<UAVArmSystem, LogicStateMachineT> {
//   bool guard(UAVArmSystem &robot_system_) {
//     Position tracking_vector;
//     return (robot_system_.getTrackingVector(tracking_vector) &&
//             robot_system_.enabled());
//   }
// };

// /**
// * @brief set noise flag to quad polynomial reference controller
// *
// * @tparam LogicStateMachineT Logic state machine used to process events
// */
// template <class LogicStateMachineT, bool flag>
// struct SetNoisePolynomialReference_
//     : EventAgnosticActionFunctor<UAVArmSystem, LogicStateMachineT> {
//   void run(UAVArmSystem &robot_system_) {
//     robot_system_.setNoisePolyReferenceController(flag);
//   }
// };

// /**
// * @brief set noise flag to quad polynomial reference controller
// *
// * @tparam LogicStateMachineT Logic state machine used to process events
// */
// template <class LogicStateMachineT>
// struct SetThrustMixingGain_
//     : EventAgnosticActionFunctor<UAVArmSystem, LogicStateMachineT> {
//   void run(UAVArmSystem &robot_system_) {
//     double thrust_gain = (this->state_machine_config_)
//                              .visual_servoing_state_machine_config()
//                              .pick_place_state_machine_config()
//                              .object_pickup_thrust_gain();
//     LOG(INFO) << "Setting thrust gain to " << thrust_gain;
//     robot_system_.setThrustMixingGain(thrust_gain);
//   }
// };

// /**
// * @brief set noise flag to quad polynomial reference controller
// *
// * @tparam LogicStateMachineT Logic state machine used to process events
// */
// template <class LogicStateMachineT>
// struct ResetThrustMixingGain_
//     : EventAgnosticActionFunctor<UAVArmSystem, LogicStateMachineT> {
//   void run(UAVArmSystem &robot_system_) {
//     LOG(INFO) << "Resetting thrust mixing gain";
//     robot_system_.resetThrustMixingGain();
//   }
// };

// /**
// * @brief set noise flag to quad polynomial reference controller
// *
// * @tparam LogicStateMachineT Logic state machine used to process events
// */
// template <class LogicStateMachineT>
// struct ResetToleranceReferenceController_
//     : EventAgnosticActionFunctor<UAVArmSystem, LogicStateMachineT> {
//   void run(UAVArmSystem &robot_system_) {
//     LOG(INFO) << "Resetting thrust mixing gain";
//     robot_system_.resetReferenceControllerTolerance();
//   }
// };

// /**
//  * @brief Set arm goal and set grip to false to start with.
//  *
//  * @tparam LogicStateMachineT State machine that contains the functor
//  * @tparam TransformIndex Index of goal transform
//  */
// template <class LogicStateMachineT, int TransformIndex>
// struct VisualServoingArmTransitionActionFunctor_
//     : EventAgnosticActionFunctor<UAVArmSystem, LogicStateMachineT> {
//   void run(UAVArmSystem &robot_system) {
//     VLOG(1) << "Setting Goal for visual servoing arm connector!";
//     auto goal =
//         this->state_machine_config_.visual_servoing_state_machine_config()
//             .pick_place_state_machine_config()
//             .arm_goal_transform()
//             .Get(TransformIndex);
//     robot_system.setGoal<VisualServoingControllerArmConnector, tf::Transform>(
//         conversions::protoTransformToTf(goal));
//     // Also ensure the gripper is in the right state to grip objects
//     robot_system.resetGripper();
//   }
// };

// /**
//  * @brief Set arm goal and set grip to false to start with.
//  *
//  * @tparam LogicStateMachineT State machine that contains the functor
//  * @tparam TransformIndex Index of goal transform
//  */
// template <class LogicStateMachineT, int TransformIndex,
//           bool ResetGripper = true, bool pickPlaceFlag = true>
// struct ArmPoseTransitionActionFunctor_
//     : EventAgnosticActionFunctor<UAVArmSystem, LogicStateMachineT> {
//   void run(UAVArmSystem &robot_system) {
//     VLOG(1) << "Setting goal pose for arm!";
//     config::Transform goal;
//     if (pickPlaceFlag) {
//       goal =
//           this->state_machine_config_.visual_servoing_state_machine_config()
//               .pick_place_state_machine_config()
//               .arm_goal_transform()
//               .Get(TransformIndex);
//     } else {
//       goal =
//           this->state_machine_config_.visual_servoing_state_machine_config()
//               .sensor_place_state_machine_config()
//               .arm_goal_transform()
//               .Get(TransformIndex);
//     }
//     robot_system.setGoal<BuiltInPoseControllerArmConnector, tf::Transform>(
//         conversions::protoTransformToTf(goal));
//     if (ResetGripper) {
//       // Also ensure the gripper is in the right state to grip objects
//       robot_system.resetGripper();
//     }
//   }
// };
// // \todo Matt Add guard for arm pose goal that checks goal index

// /**
// * @brief Action functor that attempts to pick
// * @tparam LogicStateMachineT Type of state machine
// */
// template <class LogicStateMachineT>
// struct WaitingForPickInternalActionFunctor_
//     : StateDependentInternalActionFunctor<UAVArmSystem, LogicStateMachineT,
//                                           WaitingForPick_<LogicStateMachineT>> {
//   bool run(UAVArmSystem &robot_system, LogicStateMachineT &logic_state_machine,
//            WaitingForPick_<LogicStateMachineT> &state) {
//     logic_state_machine.process_event(pick_place_events::Pick());
//     return false;
//   }
// };

/**
* @brief Action to reach a relative waypoint specified in NWU frame
* attached to quadrotor.
*
* @tparam LogicStateMachineT Logic state machine used to process events
* @tparam Index Which waypoint we are reaching to
* \todo Gowtham test internal action functor
*/
template <class LogicStateMachineT, int StartIndex, int EndIndex, class StateT, bool CheckForGoal>
struct GoToRelativeWaypointInternalActionFunctorWithObject_
    : StateDependentInternalActionFunctor<UAVArmSystem, LogicStateMachineT,
                                          StateT> {

  /**
   * @brief Specific run implementation for the internal action. The internal
   * action checks for the waypoint reaching status and updates the tracked
   * waypoint if the current waypoint is reached. When the action reaches
   * the last waypoint, the function produces a Completed event.
   *
   * @param robot_system Robot system to get waypoints
   * @param logic_state_machine State machine to process events
   * @param state  Get the current waypoint index and the last waypoint
   *
   * @return false if it processed any events. True otherwise.
   */
  bool run(UAVArmSystem &robot_system, LogicStateMachineT &logic_state_machine,
           StateT &state) {
    // Initialize controller
    if (!state.controlInitialized()) {
      PositionYaw waypoint;
      if (!state.nextWaypoint(waypoint)) {
        LOG(WARNING) << "Tracked index not available: "
                     << state.getTrackedIndex();
        logic_state_machine.process_event(be::Abort());
        return false;
      } else {
        sendLocalWaypoint(robot_system, state, waypoint, state.getReferenceConfig(),
                          state.getPositionToleranceConfig(), true);
      }
    }
    // check controller status
    ControllerStatus status = robot_system.getStatus<
        RPYTBasedReferenceConnector<Eigen::VectorXd, Eigen::VectorXd>>();
    int tracked_index = state.getTrackedIndex();
    if (status == ControllerStatus::Completed) {
      VLOG(1) << "Reached goal for tracked index: " << tracked_index;
      if (tracked_index == EndIndex) {
        // Check if next goal is in view, otherwise reset
        if (CheckForGoal && !trackingIDAvailable(state.completedEvent(), robot_system))
        {
          logic_state_machine.process_event(Completed());
        }
        else
        {
          logic_state_machine.process_event(state.completedEvent());
        }
        return false;
      } else {
        PositionYaw waypoint;
        if (!state.nextWaypoint(waypoint)) {
          LOG(WARNING) << "Tracked index not available: " << tracked_index;
          logic_state_machine.process_event(be::Abort());
          return false;
        } else {
          sendLocalWaypoint(robot_system, state, waypoint, state.getReferenceConfig(),
                            state.getPositionToleranceConfig(), false);
        }
      }
    } else if (status == ControllerStatus::Critical) {
      LOG(WARNING)
          << "Controller critical for "
          << typeid(
                 RPYTBasedReferenceConnector<Eigen::VectorXd, Eigen::VectorXd>)
                 .name();
      logic_state_machine.process_event(be::Abort());
      return false;
    } else if (status == ControllerStatus::NotEngaged) {
      LOG(WARNING)
          << "Controller not engaged for "
          << typeid(
                 RPYTBasedReferenceConnector<Eigen::VectorXd, Eigen::VectorXd>)
                 .name();
      logic_state_machine.process_event(be::Abort());
      return false;
    }
    return true;
  }

  /**
  * @brief Send local waypoint to the robot system
  * @param robot_system Robot to send waypoint to
  * @param way_point Waypoint to send
  * @param relative_to_current_pose If the waypoint should be relative to the current 
  * pose or relative to the last waypoint's goal
  */
  void sendLocalWaypoint(UAVArmSystem &robot_system, StateT &state,
                         PositionYaw way_point,
                         PolynomialReferenceConfig reference_config,
                         PositionControllerConfig goal_tolerance,
                         bool relative_to_current_pose) {
    if (relative_to_current_pose)
    {
      tf::StampedTransform quad_pose = robot_system.getPose();
      way_point.x += quad_pose.getOrigin().x();
      way_point.y += quad_pose.getOrigin().y();
      way_point.z += quad_pose.getOrigin().z();
    }
    else // relative to last pose
    {
      PositionYaw last_waypoint_world_pose = state.getLastWaypointWorldPose();
      way_point.x += last_waypoint_world_pose.x;
      way_point.y += last_waypoint_world_pose.y;
      way_point.z += last_waypoint_world_pose.z;
    }
    state.setLastWaypointWorldPose(way_point);
    VLOG(1) << "Waypoint position: " << way_point.x << ", " << way_point.y
            << ", " << way_point.z;
    tf::StampedTransform start_pose = robot_system.getPose();
    PositionYaw start_position_yaw;
    conversions::tfToPositionYaw(start_position_yaw, start_pose);
    ReferenceTrajectoryPtr<Eigen::VectorXd, Eigen::VectorXd> reference(
        new PolynomialReferenceTrajectory(way_point, start_position_yaw,
                                          reference_config));
    robot_system.setReferenceControllerTolerance(goal_tolerance);
    robot_system
        .setGoal<RPYTBasedReferenceConnector<Eigen::VectorXd, Eigen::VectorXd>,
                 ReferenceTrajectoryPtr<Eigen::VectorXd, Eigen::VectorXd>>(
            reference);
  }

  /**
  * @brief Initializing relative pose visual servoing for specific
  * tracking id based on event, check if goal exists
  *
  * @tparam LogicStateMachineT Logic state machine used to process events
  */
  bool trackingIDAvailable(ObjectId const &event, UAVArmSystem &robot_system) {
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
 * @brief FollowingWaypointSequence while checking if object is still grasped
 *
 * @tparam LogicStateMachineT logic state machine to process events
 * @tparam StartIndex starting index of relative waypoints in
 * uav_arm_system_config
 * @tparam EndIndex ending index of relative waypoints in uav_arm_system_config
 */
template <class LogicStateMachineT, int StartIndex, int EndIndex, bool CheckForGoal>
struct FollowingWaypointSequenceWithObject_
    : public BaseState<UAVArmSystem, LogicStateMachineT, msmf::none> {

  /**
   * @brief actions to be taken as internal actions when following waypoints
   */
  using WaypointActionSequenceWithObject =
      boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
          UAVStatusInternalActionFunctor_<LogicStateMachineT>,
          GoToRelativeWaypointInternalActionFunctorWithObject_<
              LogicStateMachineT, StartIndex, EndIndex,
              FollowingWaypointSequenceWithObject_<LogicStateMachineT, StartIndex,
                                         EndIndex, CheckForGoal>, CheckForGoal>,
          GripMaintainInternalActionFunctor_<LogicStateMachineT>>>;

  /**
   * @brief Internal actions, events and guards are listed here
   */
  struct internal_transition_table
      : boost::mpl::vector<msmf::Internal<InternalTransitionEvent,
                                          WaypointActionSequenceWithObject, msmf::none>> {
  };

/**
  * @brief Return the event that should be used when the waypoint sequence is
  * complete
  * @return The completed event
  */
  ObjectId completedEvent() { return object_id_; }

  /**
   * @brief set specified waypoint to robot system and store the
   * index as the one being tracked
   *
   * @param robot_system Robot system to set waypoint
   * @param tracked_index waypoint index to set and store
   * @return True if successfully set waypoint, false otherwise
   */
  bool nextWaypoint(PositionYaw &next_wp) {
    if (!control_initialized_) {
      control_initialized_ = true;
    } else {
      if (tracked_index_ + 1 < 0 ||
          tracked_index_ + 1 >= config_.way_points().size()) {
        return false;
      }
      tracked_index_++;
    }
    next_wp = conversions::protoPositionYawToPositionYaw(
        config_.way_points().Get(tracked_index_));

    return true;
  }

  /**
   * @brief store last waypoint's world pose
   *
   * @param last_wp_world World pose of last waypoint
   */
  void setLastWaypointWorldPose(PositionYaw last_wp_world) {
    last_waypoint_world_pose_ = last_wp_world;
  }

  /**
   * @brief get the last waypoint's world pose
   */
  PositionYaw getLastWaypointWorldPose() {
    if (tracked_index_ == StartIndex)
    {
      LOG(WARNING) << "WARNING: USING ZERO WAYPOINT FOR RELATIVE POSE";
    } 
    return last_waypoint_world_pose_;
  }

  /**
   * @brief Get state configuration from the state machine
   * @return state config
   */
  template <class FSM>
  FollowingWaypointSequenceConfig getConfig(FSM &logic_state_machine) {
    return logic_state_machine.configMap()
        .template find<FollowingWaypointSequenceWithObject_<LogicStateMachineT, StartIndex,
                                         EndIndex, CheckForGoal>,
              FollowingWaypointSequenceConfig>();
  }

  /**
   * @brief Function to set the starting waypoint when entering this state
   *
   * @tparam Event Event causing the entry of this state
   * @tparam FSM Logic statemachine back end
   * @param logic_state_machine state machine that processes events
   */
  template <class Event, class FSM>
  void on_entry(Event const &e, FSM &logic_state_machine) {
    BaseState<UAVArmSystem, LogicStateMachineT, msmf::none>::on_entry(
        e, logic_state_machine);
    object_id_ = e;
    config_ = this->getConfig(logic_state_machine);
    if (StartIndex >= config_.way_points().size() || StartIndex < 0) {
      LOG(WARNING) << " Starting index not in waypoint vector list";
      logic_state_machine.process_event(be::Abort());
    } else {
      tracked_index_ = StartIndex;
      control_initialized_ = false;
    }
  }

  /**
   * @brief Get current waypoint index being tracked
   *
   * @return index of waypoint being tracked
   */
  int getTrackedIndex() { return tracked_index_; }

  /**
  * @brief Whether control has been initialized or not
  * @return True if initialized, false otherwise
  */
  bool controlInitialized() { return control_initialized_; }

  PolynomialReferenceConfig getReferenceConfig() {
    return config_.poly_reference_config();
  }

  PositionControllerConfig getPositionToleranceConfig() {
    return config_.position_controller_config();
  }

private:
  FollowingWaypointSequenceConfig config_; ///< State config
  int tracked_index_ = StartIndex;         ///< Current tracked index
  PositionYaw last_waypoint_world_pose_ = PositionYaw(0,0,0,0);
  bool control_initialized_ =
      false; ///< Flag to indicate if control is initialized
  /**
   * @brief id of the package that was picked up
   */
  ObjectId object_id_;
};

/**
 * @brief State when reaching the post pick waypoint.  Stores the picked object
 * id so that it can be sent to the next state
 * @tparam LogicStateMachineT logic state machine to process events
 * @tparam StartIndex starting index of relative waypoints in
 * uav_arm_system_config
 * @tparam EndIndex ending index of relative waypoints in uav_arm_system_config
 */
template <class LogicStateMachineT, int StartIndex, int EndIndex>
struct ReachingPostPickWaypointWithObject_
    : public FollowingWaypointSequenceWithObject_<LogicStateMachineT, StartIndex,
                                        EndIndex, false> {

  /**
  * @brief Return the event that should be used when the waypoint sequence is
  * complete
  * @return The stored object id event
  */
  ObjectId completedEvent() { return object_id_; }

  /**
   * @brief Function to set the starting waypoint and to store picked object id
   * when entering this state
   *
   * @tparam FSM Logic statemachine back end
   * @param e event triggering transition
   * @param logic_state_machine state machine that processes events
   */
  template <class FSM>
  void on_entry(ObjectId const &e, FSM &logic_state_machine) {
    FollowingWaypointSequenceWithObject_<LogicStateMachineT, StartIndex, EndIndex,
                               false>::on_entry(e, logic_state_machine);
    object_id_ = e;
    SetThrustMixingGain_<FSM>()(e, logic_state_machine, *this, *this);
  }

  // On exit reset thrust gain
  template <class EventT, class FSM>
  void on_exit(EventT &e, FSM &logic_state_machine) {
    ResetThrustMixingGain_<FSM>()(e, logic_state_machine, *this, *this);
    ResetToleranceReferenceController_<FSM>()(e, logic_state_machine, *this,
                                              *this);
  }

private:
  /**
   * @brief id of the package that was picked up
   */
  ObjectId object_id_;
};

/**
 * @brief State for searching with object.  Stores the picked object
 * id so that it can be sent to the next state
 * @tparam LogicStateMachineT logic state machine to process events
 * @tparam StartIndex starting index of relative waypoints in
 * uav_arm_system_config
 * @tparam EndIndex ending index of relative waypoints in uav_arm_system_config
 */
template <class LogicStateMachineT, int StartIndex, int EndIndex>
struct SearchingWithObject_
    : public FollowingWaypointSequenceWithObject_<LogicStateMachineT, StartIndex,
                                        EndIndex, true> {

  /**
  * @brief Return the event that should be used when the waypoint sequence is
  * complete
  * @return The stored object id event
  */
  ObjectId completedEvent() { return object_id_; }

  /**
   * @brief Function to set the starting waypoint and to store picked object id
   * when entering this state
   *
   * @tparam FSM Logic statemachine back end
   * @param e event triggering transition
   * @param logic_state_machine state machine that processes events
   */
  template <class FSM>
  void on_entry(ObjectId const &e, FSM &logic_state_machine) {
    FollowingWaypointSequenceWithObject_<LogicStateMachineT, StartIndex, EndIndex,
                               true>::on_entry(e, logic_state_machine);
    object_id_ = e;
    // SetThrustMixingGain_<FSM>()(e, logic_state_machine, *this, *this);
  }

  // On exit reset thrust gain
  template <class EventT, class FSM>
  void on_exit(EventT &e, FSM &logic_state_machine) {
    // ResetThrustMixingGain_<FSM>()(e, logic_state_machine, *this, *this);
    // ResetToleranceReferenceController_<FSM>()(e, logic_state_machine, *this,
    //                                           *this);
  }

private:
  /**
   * @brief id of the package that was picked up
   */
  ObjectId object_id_;
};

// /**
// * @brief State where robot waits for an object to appear before transitioning
// * @tparam LogicStateMachineT Logic state machine used to process events
// */
// template <class LogicStateMachineT>
// struct WaitingForPick_
//     : public TimedState<
//           UAVArmSystem, LogicStateMachineT,
//           boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
//               UAVStatusInternalActionFunctor_<LogicStateMachineT>,
//               ArmStatusInternalActionFunctor_<LogicStateMachineT>,
//               WaitingForPickInternalActionFunctor_<LogicStateMachineT>>>> {};

/**
* @brief Internal action to check magnetometer. Resets only when there are critical errors.
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct MagnetometerInternalActionFunctor_
    : InternalActionFunctor<UAVSystem, LogicStateMachineT> {
  /**
  * @brief Checks if mag ratio is above theshold and aborts if so
  *
  * @param robot_system robot system to get sensor data
  * @param logic_state_machine logic state machine to trigger events
  */
  bool run(UAVSystem &robot_system, LogicStateMachineT &logic_state_machine) {
    parsernode::common::quaddata data = robot_system.getUAVData();
    if (robot_system.getConfiguration().check_mag_ratio()) {
      if (data.magdata.x > robot_system.getConfiguration().mag_ratio_threshold()) {
        LOG(WARNING) << "WARNING: Mag ratio is above threshold: " << data.magdata.x;
        logic_state_machine.process_event(Reset());
      }
    }
    return true;
  }
};

/**
* @brief Internal action functor for picking.  Resets only
* when there are critical errors
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct PickPositionControllerStatusCheck_
    : InternalActionFunctor<UAVArmSystem, LogicStateMachineT> {

  bool run(UAVArmSystem &robot_system,
           LogicStateMachineT &logic_state_machine) {
    auto connector_type = logic_state_machine.base_state_machine_config_
                              .visual_servoing_state_machine_config()
                              .connector_type();
    ControllerStatus visual_servoing_status, lowlevel_status;
    switch (connector_type) {
    case VisualServoingStateMachineConfig::RPYTPose:
      visual_servoing_status = ControllerStatus(ControllerStatus::Completed);
      lowlevel_status =
          robot_system.getStatus<RPYTRelativePoseVisualServoingConnector>();
      break;
    case VisualServoingStateMachineConfig::RPYTRef:
      visual_servoing_status = robot_system.getStatus<
          UAVVisionSystem::RPYTVisualServoingReferenceConnectorT>();
      lowlevel_status = robot_system.getStatus<
          RPYTBasedReferenceConnector<Eigen::VectorXd, Eigen::VectorXd>>();
      break;
    case VisualServoingStateMachineConfig::MPC:
      visual_servoing_status = robot_system.getStatus<
          UAVVisionSystem::MPCVisualServoingReferenceConnectorT>();
      lowlevel_status = robot_system.getStatus<MPCControllerQuadConnector>();
      break;
    case VisualServoingStateMachineConfig::VelPose:
      visual_servoing_status = ControllerStatus(ControllerStatus::Completed);
      lowlevel_status =
          robot_system
              .getStatus<RelativePoseVisualServoingControllerDroneConnector>();
      break;
    case VisualServoingStateMachineConfig::HeadingDepth:
      visual_servoing_status = ControllerStatus(ControllerStatus::Completed);
      lowlevel_status =
          robot_system.getStatus<VisualServoingControllerDroneConnector>();
      break;
    }

    // When the marker is blocked (hopefully by the gripper) or the controllers have completed
    if (visual_servoing_status == ControllerStatus::Completed ||
        lowlevel_status == ControllerStatus::Completed)
    {
      // Send ID of object
      uint32_t tracked_id;
      if (robot_system.getTrackingVectorId(tracked_id)) {
        ObjectId id_event(tracked_id);
        VLOG(1) << "Ready to grip object with ID " << tracked_id;
        logic_state_machine.process_event(id_event);
      } else {
        LOG(WARNING) << "Could not retrieve object ID!";
        logic_state_machine.process_event(Reset());
      }
      // logic_state_machine.process_event(Completed());
    }
    // lowlevel_status controller could have a warning due to continuously checking yaw or z
    // (assumes yaw and z shouldn't be changing much for pick action)
    else if (visual_servoing_status == ControllerStatus::Critical ||
                lowlevel_status == ControllerStatus::Critical ||
                lowlevel_status == ControllerStatus::NotEngaged ||
                visual_servoing_status == ControllerStatus::NotEngaged ||
                lowlevel_status.warning()) {
      VLOG(1) << "Visual servoing status: "
              << visual_servoing_status.statusAsText() << ", "
              << "Lowlevel status: " << lowlevel_status.statusAsText() << ", "
              << "Lowlevel warning: " << lowlevel_status.warning() << " " 
              << lowlevel_status.warning_description();
      robot_system.abortController(ControllerGroup::HighLevel);
      robot_system.abortController(ControllerGroup::UAV);
      logic_state_machine.process_event(Reset());
      VLOG(1) << "No controller engaged or controller "
                 "critical or warning. So resetting!";
      return false;
    }
    return true;
  }
};

/**
 * @brief Check visual servoing status and reset if something goes wrong
 *
 * @tparam LogicStateMachineT logic state machine
 */
template <class LogicStateMachineT>
using PrePickPositionInternalActionFunctor_ =
    boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
        ArmStatusInternalActionFunctor_<LogicStateMachineT>,
        VisualServoingInternalActionFunctor_<LogicStateMachineT, Reset>,
        MagnetometerInternalActionFunctor_<LogicStateMachineT>>>;

/**
 * @brief Check visual servoing status and reset if something goes wrong
 *
 * @tparam LogicStateMachineT logic state machine
 */
template <class LogicStateMachineT>
using PickPositionInternalActionFunctor_ =
    boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
        UAVStatusInternalActionFunctor_<LogicStateMachineT>,
        ArmStatusInternalActionFunctor_<LogicStateMachineT>,
        PickPositionControllerStatusCheck_<LogicStateMachineT>,
        MagnetometerInternalActionFunctor_<LogicStateMachineT>>>;


/**
* @brief Logic to check while placing object
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using PlacePositionInternalActionFunctor_ = boost::msm::front::ShortingActionSequence_<
    boost::mpl::vector<UAVStatusInternalActionFunctor_<LogicStateMachineT>,
                       ArmStatusInternalActionFunctor_<LogicStateMachineT>,
                       VisualServoingStatus_<LogicStateMachineT, Reset>,
                       MagnetometerInternalActionFunctor_<LogicStateMachineT>>>;

/**
 * @brief typedef for base class of grip state which is a timed state
 * with specified actions. The timed state provides the time from
 * entry onwards to action functors.
 *
 * @tparam LogicStateMachineT Logic state machine used to process events
 */
template <class LogicStateMachineT>
using GripBaseState_ =
    TimedState<UAVArmSystem, LogicStateMachineT,
               boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
                   UAVStatusInternalActionFunctor_<LogicStateMachineT>,
                   ArmStatusInternalActionFunctor_<LogicStateMachineT>,
                   GripInternalActionFunctor_<LogicStateMachineT>>>>;
/**
* @brief State that uses position control functor to reach a desired goal for
* picking and monitors the gripper status
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
class GripState_ : public GripBaseState_<LogicStateMachineT> {
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
        VLOG(1) << "Gripping object";
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
        VLOG(1) << "Not gripping object";
        // stop grip timer
        gripping_ = false;
      }
    }
    return grip_duration_success;
  }

  /**
   * @brief Function to set the starting waypoint when entering this state
   *
   * @tparam Event Event causing the entry of this state
   * @tparam FSM Logic statemachine back end
   * @param logic_state_machine state machine that processes events
   */
  template <class Event, class FSM>
  void on_entry(Event const &evt, FSM &logic_state_machine) {
    GripBaseState_<LogicStateMachineT>::on_entry(evt, logic_state_machine);
    grip_config_ = logic_state_machine.configMap()
                       .template find<GripState_<LogicStateMachineT>, GripConfig>();
    grip_timeout_ = std::chrono::milliseconds(grip_config_.grip_timeout());
    gripping_ = false;
    required_grip_duration_ =
        std::chrono::milliseconds(grip_config_.grip_duration());
    VLOG(1) << "Grip timeout in milliseconds: " << grip_timeout_.count();
    VLOG(1) << "Grip duration in milliseconds: "
            << required_grip_duration_.count();
  }

  /**
   * @brief Getter for timeout during gripping
   *
   * @return timeout in milliseconds obtained from state machine config
   */
  std::chrono::milliseconds gripTimeout() { return grip_timeout_; }

  /**
   * @brief Getter for grip config
   * @return The state config
   */
  const GripConfig &gripConfig() const { return grip_config_; }

private:
  /**
   * @brief Time when gripping started
   */
  std::chrono::time_point<std::chrono::high_resolution_clock> grip_start_time_;
  /**
   * @brief How long to grip before grip succeeds
   */
  std::chrono::milliseconds required_grip_duration_ =
      std::chrono::milliseconds(0);
  /**
   * @brief Flag to indicate whether gripping is active or not
   */
  bool gripping_ = false;
  /**
   * @brief Config specifying the grip settings such as duration
   */
  GripConfig grip_config_;
  /**
   * @brief Time after which grip automatically fails
   */
  std::chrono::milliseconds grip_timeout_ = std::chrono::milliseconds(0);
};

template <class LogicStateMachineT>
using PrePickPositionState_ =
    BaseState<UAVArmSystem, LogicStateMachineT,
              PrePickPositionInternalActionFunctor_<LogicStateMachineT>>;

template <class LogicStateMachineT>
using PickPositionState_ =
    BaseState<UAVArmSystem, LogicStateMachineT,
              PickPositionInternalActionFunctor_<LogicStateMachineT>>;

/**
* @brief State that uses visual servoing to place object.
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using PlacePositionState_ = BaseState<UAVArmSystem, LogicStateMachineT,
                              PlacePositionInternalActionFunctor_<LogicStateMachineT>>;

template <class LogicStateMachineT>
struct PrePlacePositionState_ : public PlacePositionState_<LogicStateMachineT> {};