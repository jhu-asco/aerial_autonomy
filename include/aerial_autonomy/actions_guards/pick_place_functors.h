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

// Forward declaration for ReachingWaypointInternalActionFunctor_
template <class LogicStateMachineT, int StartIndex, int EndIndex>
struct FollowingWaypointSequence_;
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
    VLOG(1) << "Monitoring grip";
    bool has_grip = robot_system.grip(true);
    if (has_grip) {
      VLOG(1) << "Gripping object";
    }
    if (state.monitorGrip(has_grip)) {
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

template <class LogicStateMachineT>
struct PickTransitionGuardFunctor_
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
            RelativePoseVisualServoingControllerDroneConnector, false, Reset>>>;

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
            RelativePoseVisualServoingControllerDroneConnector, true, Reset>>>;

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
struct PrePickTransitionGuardFunctor_
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
* @brief Action to reach a relative waypoint specified in NWU frame
* attached to quadrotor.
*
* @tparam LogicStateMachineT Logic state machine used to process events
* @tparam Index Which waypoint we are reaching to
* \todo Gowtham test internal action functor
*/
template <class LogicStateMachineT, int StartIndex, int EndIndex>
struct GoToWaypointInternalActionFunctor_
    : StateDependentInternalActionFunctor<
          UAVArmSystem, LogicStateMachineT,
          FollowingWaypointSequence_<LogicStateMachineT, StartIndex,
                                     EndIndex>> {
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
           FollowingWaypointSequence_<LogicStateMachineT, StartIndex, EndIndex>
               &state) {
    // check controller status
    ControllerStatus status =
        robot_system.getStatus<VelocityBasedPositionControllerDroneConnector>();
    int tracked_index = state.getTrackedIndex();
    if (status == ControllerStatus::Completed) {
      VLOG(1) << "Reached goal for tracked index: " << tracked_index;
      if (tracked_index == EndIndex) {
        logic_state_machine.process_event(Completed());
        return false;
      } else {
        tracked_index += 1;
        if (!robot_system.checkWaypointIndex(tracked_index)) {
          LOG(WARNING) << "Tracked index not available: " << tracked_index;
          logic_state_machine.process_event(be::Abort());
          return false;
        } else {
          state.setWaypoint(robot_system, tracked_index);
        }
      }
    } else if (status == ControllerStatus::Critical) {
      LOG(WARNING)
          << "Controller critical for "
          << typeid(VelocityBasedPositionControllerDroneConnector).name();
      logic_state_machine.process_event(be::Abort());
      return false;
    }
    return true;
  }
};

/**
* @brief Guard for waypoint A transition
*
* @tparam LogicStateMachineT Logic state machine used to process events
* @tparam StartIndex starting waypoint index to follow
* @tparam EndIndex ending waypoint index to follow
*/
template <class LogicStateMachineT, int StartIndex, int EndIndex>
struct WaypointSequenceTransitionGuardFunctor_
    : EventAgnosticGuardFunctor<UAVArmSystem, LogicStateMachineT> {
  bool guard(UAVArmSystem &robot_system) {
    bool result = robot_system.checkWaypointIndex(StartIndex) &&
                  robot_system.checkWaypointIndex(EndIndex);
    if (!result) {
      LOG(WARNING) << "StartIndex: " << StartIndex << " or EndIndex "
                   << EndIndex
                   << " not available in waypoint vector in config file";
    }
    return result;
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
template <class LogicStateMachineT, int StartIndex, int EndIndex>
struct FollowingWaypointSequence_
    : public BaseState<UAVArmSystem, LogicStateMachineT, msmf::none> {

  /**
   * @brief actions to be taken as internal actions when following waypoints
   */
  using WaypointActionSequence = boost::msm::front::ShortingActionSequence_<
      boost::mpl::vector<UAVStatusInternalActionFunctor_<LogicStateMachineT>,
                         GoToWaypointInternalActionFunctor_<
                             LogicStateMachineT, StartIndex, EndIndex>>>;

  /**
   * @brief Internal actions, events and guards are listed here
   */
  struct internal_transition_table
      : boost::mpl::vector<msmf::Internal<InternalTransitionEvent,
                                          WaypointActionSequence, msmf::none>> {
  };

  /**
   * @brief set specified waypoint to robot system and store the
   * index as the one being tracked
   *
   * @param robot_system Robot system to set waypoint
   * @param tracked_index waypoint index to set and store
   */
  void setWaypoint(UAVArmSystem &robot_system, int tracked_index) {
    tracked_index_ = tracked_index;
    PositionYaw way_point = robot_system.getWaypoint(tracked_index);
    parsernode::common::quaddata data = robot_system.getUAVData();
    way_point.x += data.localpos.x;
    way_point.y += data.localpos.y;
    way_point.z += data.localpos.z;
    VLOG(1) << "Going to waypoint " << tracked_index;
    VLOG(1) << "Waypoint position: " << way_point.x << ", " << way_point.y
            << ", " << way_point.z;
    robot_system.setGoal<VelocityBasedPositionControllerDroneConnector,
                         PositionYaw>(way_point);
  }

  /**
   * @brief Function to set the starting waypoint when entering this state
   *
   * @tparam Event Event causing the entry of this state
   * @tparam FSM Logic statemachine back end
   * @param logic_state_machine state machine that processes events
   */
  template <class Event, class FSM>
  void on_entry(Event const &, FSM &logic_state_machine) {
    UAVArmSystem &robot_system = this->getRobotSystem(logic_state_machine);
    if (robot_system.checkWaypointIndex(StartIndex)) {
      setWaypoint(robot_system, StartIndex);
    } else {
      LOG(WARNING) << " Starting index not in waypoint vector list";
      logic_state_machine.process_event(be::Abort());
    }
  }

  /**
   * @brief Get current waypoint index being tracked
   *
   * @return index of waypoint being tracked
   */
  int getTrackedIndex() { return tracked_index_; }

private:
  int tracked_index_ = StartIndex; ///< Current tracked index
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
* @brief State that uses visual servoing to place object.
*
* @tparam LogicStateMachineT Logic state machine used to process events
*/
template <class LogicStateMachineT>
using PlaceState_ = BaseState<UAVArmSystem, LogicStateMachineT,
                              PlaceInternalActionFunctor_<LogicStateMachineT>>;

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
