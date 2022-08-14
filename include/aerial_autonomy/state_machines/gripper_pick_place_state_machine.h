#pragma once
/**
 * GripperPickPlace State Machine that handles flying, going to an object of interest
 * and pick/place the object.
 * Built for a fixed arm with a gripper attached to the end. 
 *
 * Events:
 *      >>> Without args
 *      Land
 *      Takeoff
 *      Abort
 *      Pick
 *      Place
 *      GoHome
 *      >>> With args
 *      PositionYaw
 */

// back-end
#include <aerial_autonomy/common/thread_safe_state_machine.h>

// front-end
#include <boost/msm/front/state_machine_def.hpp>

// functors
#include <boost/msm/front/functor_row.hpp>

// Actions and guards used
#include <aerial_autonomy/actions_guards/gripper_pick_place_states_actions.h>

// Robot System used
#include <aerial_autonomy/robot_systems/uav_arm_system.h>

// Logging library
#include <glog/logging.h>

// Store state machine states
#include <array>

// Base state machine
#include <aerial_autonomy/state_machines/base_state_machine.h>

// Events
#include <aerial_autonomy/pick_place_events.h>

#include <aerial_autonomy/types/manual_control_event.h>

namespace msmf = boost::msm::front;
namespace vse = visual_servoing_events;
namespace pe = pick_place_events;
namespace be = uav_basic_events;

// Forward Declaration
struct GripperPickPlaceStateMachineFrontEnd;

/**
* @brief Backend for Logic State Machine.
*
* Used to forward arguments to constructor, and process events
*/
using GripperPickPlaceStateMachine =
    boost::msm::back::thread_safe_state_machine<GripperPickPlaceStateMachineFrontEnd>;

/**
* @brief Namespace for basic uav states and actions such as takeoff, land etc
*/
using gsa = GripperPickPlaceStatesActions<GripperPickPlaceStateMachine>;

/**
* @brief front-end: define the FSM structure
*
* This state machine defines the logic for a multi-destination
* pick-and-place task.  After taking off, the robot waits
* until it sees an object.  When it sees an object, it will attempt to pick it
* up.
* If it fails to pick up the object it returns to home and tries again.
* If it succeeds, the robot will take the object to a configured destination
* based on the ID of the object it picked.  The robot will then move back to
* the starting point and wait until it sees another object.
*/
class GripperPickPlaceStateMachineFrontEnd
    : public msmf::state_machine_def<GripperPickPlaceStateMachineFrontEnd>,
      public BaseStateMachine<UAVArmSystem> {
public:
  /**
  * @brief Action to take on entering state machine
  *
  * @tparam Event type of event causing state machine to enter
  * @tparam FSM Backend finite state machine type to trigger events
  */
  template <class Event, class FSM> void on_entry(Event const &evt, FSM &fsm) {
    VLOG(1) << "entering: PickPlace system";
    int dummy_source_state = 0, dummy_target_state = 0;
    gsa::ArmPowerOn()(evt, fsm, dummy_source_state, dummy_target_state);
  }
  /**
  * @brief Action to take on leaving state machine
  *
  * @tparam Event type of event causing state machine to enter
  * @tparam FSM Backend finite state machine type to trigger events
  */
  template <class Event, class FSM> void on_exit(Event const &evt, FSM &fsm) {
    VLOG(1) << "leaving: PickPlace system";
    int dummy_source_state = 0, dummy_target_state = 0;
    gsa::ArmPowerOff()(evt, fsm, dummy_source_state, dummy_target_state);
  }

  /**
  * @brief Constructor with arguments to store robot system
  *
  * @param uav_system robot system that is stored internally
  * and shared with events
  * @param state_machine_config store config variables for state
  * machine
  */
  GripperPickPlaceStateMachineFrontEnd(
      UAVArmSystem &uav_system,
      const BaseStateMachineConfig &state_machine_config)
      : BaseStateMachine(uav_system, state_machine_config) {
    auto pick_state_machine_config =
        state_machine_config.visual_servoing_state_machine_config()
            .pick_place_state_machine_config();
    config_map_.insert<gsa::ReachingPostPickWaypointWithObjectBase>(
        pick_state_machine_config.following_waypoint_sequence_config());
    config_map_.insert<gsa::ReachingPostPlaceWaypoint>(
        pick_state_machine_config.following_waypoint_sequence_config());
    config_map_.insert<gsa::GripState>(pick_state_machine_config.grip_config());
  }

  /**
   * @brief Alternative constructor with default state machine config
   *
   * @param uav_system robot system that is stored internally and shared with
   * events
   */
  GripperPickPlaceStateMachineFrontEnd(UAVArmSystem &uav_system)
      : GripperPickPlaceStateMachineFrontEnd(uav_system, BaseStateMachineConfig()){};

  /**
  * @brief Initial state for state machine
  */
  using initial_state = gsa::ManualControlArmState;
  /**
  * @brief Transition table for State Machine
  */
  struct transition_table
      : boost::mpl::vector<
            //        Start          Event         Next           Action Guard
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<gsa::Landed, be::Takeoff, gsa::TakingOff,
                      gsa::TakeoffAction, gsa::TakeoffGuard>,
            msmf::Row<gsa::Landed, ManualControlEvent,
                      gsa::ManualControlArmState, msmf::none, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<gsa::TakingOff, Completed, gsa::Hovering,
                      gsa::HoverInPlaceTransitionAction, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<gsa::Hovering, PositionYaw, gsa::ReachingGoal,
                      gsa::AbortControllerReachingGoalSet, gsa::ReachingGoalGuard>,
            msmf::Row<gsa::Hovering, VelocityYaw, gsa::ExecutingVelocityGoal,
                      gsa::AbortControllerSetVelocityGoal, gsa::GuardVelocityGoal>,
            msmf::Row<gsa::Hovering, be::Land, gsa::Landing,
                      gsa::AbortControllerLandingAction, msmf::none>,
            msmf::Row<gsa::Hovering, ManualControlEvent, gsa::ManualControlArmState, 
                      gsa::UAVControllerAbort, msmf::none>,
            msmf::Row<gsa::Hovering, pe::Pick, gsa::WaitingForPick, 
                      gsa::UAVControllerAbort, msmf::none>,
            msmf::Row<gsa::Hovering, vse::GoHome, gsa::ReachingGoal,
                      gsa::AbortControllerGoHomeTransitionAction, gsa::GoHomeTransitionGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<gsa::ReachingGoal, be::Abort, gsa::Hovering,
                      gsa::AbortUAVArmControllerHoverInPlace, msmf::none>,
            msmf::Row<gsa::ReachingGoal, Completed, gsa::Hovering,
                      gsa::AbortUAVArmControllerHoverInPlace, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<gsa::ExecutingVelocityGoal, VelocityYaw, gsa::ExecutingVelocityGoal, 
                      gsa::SetVelocityGoal, gsa::GuardVelocityGoal>,
            msmf::Row<gsa::ExecutingVelocityGoal, be::Abort, gsa::Hovering,
                      gsa::AbortUAVArmControllerHoverInPlace, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<gsa::Landing, Completed, gsa::Landed, 
                      msmf::none, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<gsa::WaitingForPick, pe::Pick, gsa::PrePickPositionState,
                      gsa::PrePickPositionTransitionAction, gsa::PrePickTransitionGuard>,
            msmf::Row<gsa::WaitingForPick, be::Abort, gsa::Hovering,
                      gsa::AbortUAVArmControllerHoverInPlace, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<gsa::PrePickPositionState, Completed, gsa::PickPositionState,
                      gsa::PickPositionTransitionAction, msmf::none>,
            msmf::Row<gsa::PrePickPositionState, be::Abort, gsa::Hovering,
                      gsa::AbortUAVArmControllerHoverInPlace, msmf::none>,
            msmf::Row<gsa::PrePickPositionState, Reset, gsa::ResetVisualServoing,
                      gsa::UngripGoHome, gsa::GoHomeTransitionGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<gsa::ResetVisualServoing, Completed, gsa::WaitingForPick,
                      gsa::AbortUAVArmControllerHoverInPlace, msmf::none>,
            msmf::Row<gsa::ResetVisualServoing, be::Abort, gsa::Hovering,
                      gsa::AbortUAVArmControllerHoverInPlace, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<gsa::PickPositionState, Completed, gsa::GripState,
                      gsa::GripTransitionAction, msmf::none>,
            msmf::Row<gsa::PickPositionState, be::Abort, gsa::Hovering,
                      gsa::AbortUAVArmControllerHoverInPlace, msmf::none>,
            msmf::Row<gsa::PickPositionState, Reset, gsa::ResetVisualServoing,
                      gsa::UngripGoHome, gsa::GoHomeTransitionGuard>,         
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<gsa::GripState, be::Abort, gsa::Hovering,
                      gsa::AbortUAVArmControllerHoverInPlace, msmf::none>,
            msmf::Row<gsa::GripState, Reset, gsa::ResetVisualServoing,
                      gsa::UngripGoHome, gsa::GoHomeTransitionGuard>,
            msmf::Row<gsa::GripState, ObjectId, gsa::ReachingPostPickWaypointWithObject,
                      gsa::AbortUAVArmControllerHoverInPlace, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<gsa::ReachingPostPickWaypointWithObject, be::Abort, gsa::Hovering,
                      gsa::AbortUAVArmControllerHoverInPlace, msmf::none>,
            msmf::Row<gsa::ReachingPostPickWaypointWithObject, Reset, gsa::ResetVisualServoing,
                      gsa::UngripGoHome, gsa::GoHomeTransitionGuard>,
            msmf::Row<gsa::ReachingPostPickWaypointWithObject, ObjectId, gsa::PrePlacePositionState,
                      gsa::PrePlaceVisualServoingTransitionAction,
                      gsa::PrePlaceVisualServoingTransitionGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<gsa::PrePlacePositionState, Completed, gsa::PlacePositionState,
                      gsa::PlaceTransitionAction, gsa::PlaceVisualServoingTransitionGuard>,
            msmf::Row<gsa::PrePlacePositionState, Reset, gsa::ResetVisualServoingPlace,
                      gsa::GoHomeTransitionAction, gsa::GoHomeTransitionGuard>,
            msmf::Row<gsa::PrePlacePositionState, be::Abort, gsa::Hovering,
                      gsa::AbortUAVArmControllerHoverInPlace, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<gsa::ResetVisualServoingPlace, Completed, gsa::PrePlacePositionState,
                      gsa::PrePlaceVisualServoingTransitionAction, msmf::none>,
            msmf::Row<gsa::ResetVisualServoingPlace, be::Abort, gsa::Hovering,
                      gsa::AbortUAVArmControllerHoverInPlace, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<gsa::PlacePositionState, Completed, gsa::ReachingPostPlaceWaypoint, 
                      gsa::ArmGripAction<false>, msmf::none>,
            msmf::Row<gsa::PlacePositionState, Reset, gsa::ResetVisualServoingPlace, 
                      gsa::GoHomeTransitionAction, gsa::GoHomeTransitionGuard>,
            msmf::Row<gsa::PlacePositionState, be::Abort, gsa::Hovering,
                      gsa::AbortUAVArmControllerHoverInPlace, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<gsa::ReachingPostPlaceWaypoint, Completed, gsa::WaitingForPick, 
                      gsa::AbortUAVArmControllerHoverInPlace, msmf::none>,
            msmf::Row<gsa::ReachingPostPlaceWaypoint, be::Abort, gsa::Hovering,
                      gsa::AbortUAVArmControllerHoverInPlace, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<gsa::ManualControlArmState, be::Takeoff, gsa::Hovering,
                      gsa::ManualControlSwitchHoverInPlace, gsa::ManualControlSwitchGuard>,
            msmf::Row<gsa::ManualControlArmState, be::Land, gsa::Landed,
                      gsa::ManualControlSwitchAction, gsa::ManualControlSwitchGuard>> {};
  /**
  * @brief Use Inherited no transition function
  */
  using BaseStateMachine<UAVArmSystem>::no_transition;
};

/**
* @brief state names to get name based on state id
*/
static constexpr std::array<const char *, 17> state_names = {
    "Landed",
    "Takingoff",
    "Hovering",
    "ReachingGoal",
    "ExecutingVelocityGoal",
    "Landing",
    "WaitingForPick",
    "PrePickPositionState",
    "ResetVisualServoing",
    "PickPositionState",
    "GripState",
    "ReachingPostPickWaypointWithObject",
    "PrePlacePositionState",
    "ResetVisualServoingPlace",
    "PlacePositionState",
    "ReachingPostPlaceWaypoint",
    "ManualControlArmState"};
/**
* @brief Get current state name
*
* @param p Logic state machine backend to access current state
*
* @return state name
*/
const char *pstate(GripperPickPlaceStateMachine const &p) {
  return state_names.at(p.current_state()[0]);
}
