#pragma once
#include "grip_config.pb.h"
#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/actions_guards/hovering_functors.h>
#include <aerial_autonomy/actions_guards/manual_control_functors.h>
#include <aerial_autonomy/actions_guards/shorting_action_sequence.h>
#include <aerial_autonomy/actions_guards/visual_servoing_functors.h>
#include <aerial_autonomy/common/conversions.h>
#include <aerial_autonomy/common/proto_utils.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <aerial_autonomy/logic_states/timed_state.h>
#include <aerial_autonomy/pick_place_events.h>
#include <aerial_autonomy/robot_systems/uav_arm_system.h>
#include <aerial_autonomy/types/completed_event.h>
#include <aerial_autonomy/types/object_id.h>
#include <aerial_autonomy/types/reset_event.h>
#include <chrono>
#include <glog/logging.h>
#include <thread>

// Forward declaration for GrippingInternalActionFunctor_
template <class LogicStateMachineT> class PickState_;

// Forward declaration for ReachingWaypointInternalActionFunctor_
template <class LogicStateMachineT, int StartIndex, int EndIndex,
          class CompletedEvent>
struct FollowingWaypointSequence_;

// Forward declaration for WaitingForPickInternalActionFunctor_
template <class LogicStateMachineT> struct WaitingForPick_;

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
* @brief Logic to check while placing object
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using PlaceInternalActionFunctor_ =
    boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
        UAVStatusInternalActionFunctor_<LogicStateMachineT>,
        ArmStatusInternalActionFunctor_<LogicStateMachineT>,
        ControllerStatusInternalActionFunctor_<
            LogicStateMachineT,
            RPYTBasedReferenceConnector<Eigen::VectorXd, Eigen::VectorXd>, true,
            Reset>>>;

/**
* @brief Check tracking is valid before starting visual servoing and arm is
* enabled before picking objects
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct ArmTrackingGuardFunctor_
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
    auto goal =
        this->state_machine_config_.visual_servoing_state_machine_config()
            .pick_place_state_machine_config()
            .arm_goal_transform()
            .Get(TransformIndex);
    robot_system.setGoal<VisualServoingControllerArmConnector, tf::Transform>(
        conversions::protoTransformToTf(goal));
    // Also ensure the gripper is in the right state to grip objects
    robot_system.resetGripper();
  }
};

/**
 * @brief Set arm goal and set grip to false to start with.
 *
 * @tparam LogicStateMachineT State machine that contains the functor
 * @tparam TransformIndex Index of goal transform
 */
template <class LogicStateMachineT, int TransformIndex>
struct ArmPoseTransitionActionFunctor_
    : EventAgnosticActionFunctor<UAVArmSystem, LogicStateMachineT> {
  void run(UAVArmSystem &robot_system) {
    VLOG(1) << "Setting goal pose for arm!";
    auto goal =
        this->state_machine_config_.visual_servoing_state_machine_config()
            .pick_place_state_machine_config()
            .arm_goal_transform()
            .Get(TransformIndex);
    robot_system.setGoal<BuiltInPoseControllerArmConnector, tf::Transform>(
        conversions::protoTransformToTf(goal));
    // Also ensure the gripper is in the right state to grip objects
    robot_system.resetGripper();
  }
};
// \todo Matt Add guard for arm pose goal that checks goal index

/**
* @brief Action functor that attempts to pick
* @tparam LogicStateMachineT Type of state machine
*/
template <class LogicStateMachineT>
struct WaitingForPickInternalActionFunctor_
    : StateDependentInternalActionFunctor<UAVArmSystem, LogicStateMachineT,
                                          WaitingForPick_<LogicStateMachineT>> {
  bool run(UAVArmSystem &robot_system, LogicStateMachineT &logic_state_machine,
           WaitingForPick_<LogicStateMachineT> &state) {
    logic_state_machine.process_event(pick_place_events::Pick());
    return false;
  }
};

/**
* @brief Action to reach a relative waypoint specified in NWU frame
* attached to quadrotor.
*
* @tparam LogicStateMachineT Logic state machine used to process events
* @tparam Index Which waypoint we are reaching to
* \todo Gowtham test internal action functor
*/
template <class LogicStateMachineT, int StartIndex, int EndIndex, class StateT>
struct GoToWaypointInternalActionFunctor_
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
        sendLocalWaypoint(robot_system, waypoint);
      }
    }
    // check controller status
    ControllerStatus status = robot_system.getStatus<
        RPYTBasedReferenceConnector<Eigen::VectorXd, Eigen::VectorXd>>();
    int tracked_index = state.getTrackedIndex();
    if (status == ControllerStatus::Completed) {
      VLOG(1) << "Reached goal for tracked index: " << tracked_index;
      if (tracked_index == EndIndex) {
        logic_state_machine.process_event(state.completedEvent());
        return false;
      } else {
        PositionYaw waypoint;
        if (!state.nextWaypoint(waypoint)) {
          LOG(WARNING) << "Tracked index not available: " << tracked_index;
          logic_state_machine.process_event(be::Abort());
          return false;
        } else {
          sendLocalWaypoint(robot_system, waypoint);
        }
      }
    } else if (status == ControllerStatus::Critical) {
      LOG(WARNING) << "Controller critical for "
                   << typeid(RPYTBasedReferenceConnector).name();
      logic_state_machine.process_event(be::Abort());
      return false;
    } else if (status == ControllerStatus::NotEngaged) {
      LOG(WARNING) << "Controller not engaged for "
                   << typeid(RPYTBasedReferenceConnector).name();
      logic_state_machine.process_event(be::Abort());
      return false;
    }
    return true;
  }

  /**
  * @brief Send local waypoint to the robot system
  * @param robot_system Robot to send waypoint to
  * @param way_point Waypoint to send
  */
  void sendLocalWaypoint(UAVArmSystem &robot_system, PositionYaw way_point) {
    parsernode::common::quaddata data = robot_system.getUAVData();
    way_point.x += data.localpos.x;
    way_point.y += data.localpos.y;
    way_point.z += data.localpos.z;
    VLOG(1) << "Waypoint position: " << way_point.x << ", " << way_point.y
            << ", " << way_point.z;
    robot_system
        .setGoal<RPYTBasedReferenceConnector<Eigen::VectorXd, Eigen::VectorXd>,
                 ReferenceTrajectoryPtr<Eigen::VectorXd, Eigen::VectorXd>>(
            conversions::createWaypoint(way_point));
  }
};

/**
 * @brief State to follow a series of relative waypoints starting
 * from StartIndex and ending at EndIndex (including EndIndex).
 * The internal action checks for reaching current waypoint and
 * increments waypoint. If the state reaches the end of waypoint
 * list, the state processes completed event. The state also
 * checks for UAV status in addition to waypoint reaching
 *
 * @tparam LogicStateMachineT logic state machine to process events
 * @tparam StartIndex starting index of relative waypoints in
 * uav_arm_system_config
 * @tparam EndIndex ending index of relative waypoints in uav_arm_system_config
 */
template <class LogicStateMachineT, int StartIndex, int EndIndex,
          class CompletedEvent = Completed>
struct FollowingWaypointSequence_
    : public BaseState<UAVArmSystem, LogicStateMachineT, msmf::none> {

  /**
   * @brief actions to be taken as internal actions when following waypoints
   */
  using WaypointActionSequence =
      boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
          UAVStatusInternalActionFunctor_<LogicStateMachineT>,
          GoToWaypointInternalActionFunctor_<
              LogicStateMachineT, StartIndex, EndIndex,
              FollowingWaypointSequence_<LogicStateMachineT, StartIndex,
                                         EndIndex, CompletedEvent>>>>;

  /**
   * @brief Internal actions, events and guards are listed here
   */
  struct internal_transition_table
      : boost::mpl::vector<msmf::Internal<InternalTransitionEvent,
                                          WaypointActionSequence, msmf::none>> {
  };

  /**
  * @brief Return the event that should be used when the waypoint sequence is
  * complete
  * @return The completed event
  */
  virtual CompletedEvent completedEvent() { return CompletedEvent(); }

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
   * @brief Get state configuration from the state machine
   * @return state config
   */
  template <class FSM>
  FollowingWaypointSequenceConfig getConfig(FSM &logic_state_machine) {
    return logic_state_machine.configMap()
        .find<FollowingWaypointSequence_<LogicStateMachineT, StartIndex,
                                         EndIndex, CompletedEvent>,
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

private:
  FollowingWaypointSequenceConfig config_; ///< State config
  int tracked_index_ = StartIndex;         ///< Current tracked index
  bool control_initialized_ =
      false; ///< Flag to indicate if control is initialized
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
struct ReachingPostPickWaypoint_
    : public FollowingWaypointSequence_<LogicStateMachineT, StartIndex,
                                        EndIndex, ObjectId> {

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
    FollowingWaypointSequence_<LogicStateMachineT, StartIndex, EndIndex,
                               ObjectId>::on_entry(e, logic_state_machine);
    object_id_ = e;
  }

private:
  /**
   * @brief id of the package that was picked up
   */
  ObjectId object_id_;
};

/**
* @brief State that uses visual servoing to place object.
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using PlaceState_ = BaseState<UAVArmSystem, LogicStateMachineT,
                              PlaceInternalActionFunctor_<LogicStateMachineT>>;

/**
* @brief State where robot waits for an object to appear before transitioning
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct WaitingForPick_
    : public TimedState<
          UAVArmSystem, LogicStateMachineT,
          boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
              UAVStatusInternalActionFunctor_<LogicStateMachineT>,
              ArmStatusInternalActionFunctor_<LogicStateMachineT>,
              WaitingForPickInternalActionFunctor_<LogicStateMachineT>>>> {};

/**
* @brief Internal action functor for picking.  Resets only
* when the manipulator is not already gripping
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
struct PickControllerStatusCheck_
    : InternalActionFunctor<UAVArmSystem, LogicStateMachineT> {

  bool run(UAVArmSystem &robot_system,
           LogicStateMachineT &logic_state_machine) {
    ControllerStatus visual_servoing_status =
        robot_system
            .getStatus<UAVVisionSystem::VisualServoingReferenceConnectorT>();
    ControllerStatus lowlevel_status = robot_system.getStatus<
        RPYTBasedReferenceConnector<Eigen::VectorXd, Eigen::VectorXd>>();
    bool grip_status = robot_system.gripStatus();
    if ((visual_servoing_status == ControllerStatus::Critical ||
         lowlevel_status == ControllerStatus::Critical) &&
        grip_status) {
      robot_system.abortController(ControllerGroup::HighLevel);
      robot_system.abortController(ControllerGroup::UAV);
      VLOG(1)
          << "Controller critical while gripping is true! Aborting Controller!";
    } else if ((visual_servoing_status == ControllerStatus::Critical ||
                lowlevel_status == ControllerStatus::Critical ||
                visual_servoing_status == ControllerStatus::NotEngaged) &&
               !grip_status) {
      robot_system.abortController(ControllerGroup::HighLevel);
      logic_state_machine.process_event(Reset());
      VLOG(1) << "Gripping failed and no controller engaged or controller "
                 "critical. So resetting!";
      return false;
    }
    return true;
  }
};

/**
 * @brief typedef for base class of pick state which is a timed state
 * with specified actions. The timed state provides the time from
 * entry onwards to action functors.
 *
 * @tparam LogicStateMachineT Logic state machine used to process events
 */
template <class LogicStateMachineT>
using PickBaseState_ =
    TimedState<UAVArmSystem, LogicStateMachineT,
               boost::msm::front::ShortingActionSequence_<boost::mpl::vector<
                   UAVStatusInternalActionFunctor_<LogicStateMachineT>,
                   ArmStatusInternalActionFunctor_<LogicStateMachineT>,
                   PickControllerStatusCheck_<LogicStateMachineT>,
                   GrippingInternalActionFunctor_<LogicStateMachineT>>>>;
/**
* @brief State that uses position control functor to reach a desired goal for
* picking and monitors the gripper status
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
class PickState_ : public PickBaseState_<LogicStateMachineT> {
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
    PickBaseState_<LogicStateMachineT>::on_entry(evt, logic_state_machine);
    grip_config_ = logic_state_machine.configMap()
                       .find<PickState_<LogicStateMachineT>, GripConfig>();
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
