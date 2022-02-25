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
#include <aerial_autonomy/actions_guards/pick_place_states_actions.h>

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
using psa = PickPlaceStatesActions<GripperPickPlaceStateMachine>;

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
    psa::ArmPowerOn()(evt, fsm, dummy_source_state, dummy_target_state);
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
    psa::ArmPowerOff()(evt, fsm, dummy_source_state, dummy_target_state);
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
    config_map_.insert<psa::ReachingPostPickWaypointBase>(
        pick_state_machine_config.following_waypoint_sequence_config());
    config_map_.insert<psa::ReachingPostPlaceWaypoint>(
        pick_state_machine_config.following_waypoint_sequence_config());
    config_map_.insert<psa::PickState>(pick_state_machine_config.grip_config());
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
  using initial_state = psa::ManualControlArmState;
  /**
  * @brief Transition table for State Machine
  */
  struct transition_table
      : boost::mpl::vector<
            //        Start          Event         Next           Action Guard
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::Landed, be::Takeoff, psa::ArmPreTakeoffFolding,
                      psa::ArmPowerOnFold, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::Landed, ManualControlEvent,
                      psa::ManualControlArmState, msmf::none, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::ArmPreTakeoffFolding, Completed, psa::TakingOff,
                      psa::TakeoffAction, psa::TakeoffGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::ArmPreTakeoffFolding, be::Abort, psa::Landed,
                      psa::ArmPowerOff, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::TakingOff, Completed, psa::Hovering,
                      psa::ArmRightFold, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::Hovering, PositionYaw, psa::ReachingGoal,
                      psa::ReachingGoalSet, psa::ReachingGoalGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::Hovering, pe::Pick, psa::WaitingForPick, msmf::none,
                      msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::ResetVisualServoing, Completed, psa::WaitingForPick,
                      psa::AbortUAVArmController, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::WaitingForPick, pe::Pick, psa::PrePickState,
                      psa::PrePickTransitionAction,
                      psa::PrePickTransitionGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::PrePickState, Completed, psa::PickState,
                      psa::PickTransitionAction, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::PrePickState, be::Abort, psa::Hovering,
                      psa::AbortUAVArmController, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::PrePickState, Reset, psa::ResetVisualServoing,
                      psa::ArmRightFoldGoHome, psa::GoHomeTransitionGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::WaitingForPick, be::Abort, psa::Hovering,
                      psa::AbortUAVArmController, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::ResetVisualServoing, be::Abort, psa::Hovering,
                      psa::AbortUAVArmController, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::Hovering, VelocityYaw, psa::ExecutingVelocityGoal,
                      psa::SetVelocityGoal, psa::GuardVelocityGoal>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::Hovering, vse::GoHome, psa::ReachingGoal,
                      psa::GoHomeTransitionAction, psa::GoHomeTransitionGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::Hovering, be::Land, psa::ArmPreLandingFolding,
                      psa::ArmFold, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::ArmPreLandingFolding, Completed, psa::Landing,
                      psa::LandingAction, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::ArmPreLandingFolding, be::Abort, psa::Landing,
                      psa::LandingAction, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::Hovering, ManualControlEvent,
                      psa::ManualControlArmState, msmf::none, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::ReachingGoal, be::Abort, psa::Hovering,
                      psa::AbortUAVControllerArmRightFold, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::ExecutingVelocityGoal, VelocityYaw,
                      psa::ExecutingVelocityGoal, psa::SetVelocityGoal,
                      psa::GuardVelocityGoal>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::ExecutingVelocityGoal, be::Abort, psa::Hovering,
                      psa::AbortUAVControllerArmRightFold, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::PickState, be::Abort, psa::Hovering,
                      psa::AbortUAVArmController, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::Landing, Completed, psa::Landed, psa::ArmPowerOff,
                      msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::ReachingGoal, Completed, psa::Hovering,
                      psa::AbortUAVControllerArmRightFold, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::PickState, Reset, psa::ResetVisualServoing,
                      psa::ArmRightFoldGoHome, psa::GoHomeTransitionGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::PickState, ObjectId, psa::ReachingPostPickWaypoint,
                      psa::AbortUAVArmControllerArmRightFold, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::ReachingPostPickWaypoint, be::Abort, psa::Hovering,
                      psa::AbortUAVArmController, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::ReachingPostPickWaypoint, ObjectId,
                      psa::PrePlaceState,
                      psa::PrePlaceVisualServoingTransitionAction,
                      psa::PrePlaceVisualServoingTransitionGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::PrePlaceState, Completed, psa::PlaceState,
                      psa::PlaceVisualServoingTransitionAction,
                      psa::PlaceVisualServoingTransitionGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::PrePlaceState, be::Abort, psa::Hovering,
                      psa::AbortUAVArmController, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::PlaceState, Completed,
                      psa::ReachingPostPlaceWaypoint, psa::ArmGripAction<false>,
                      msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::ReachingPostPlaceWaypoint, Completed,
                      psa::WaitingForPick, psa::AbortUAVArmController,
                      msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::ReachingPostPlaceWaypoint, be::Abort, psa::Hovering,
                      psa::AbortUAVArmController, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::PlaceState, be::Abort, psa::Hovering,
                      psa::AbortUAVArmController, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::ManualControlArmState, be::Takeoff, psa::Hovering,
                      psa::ManualControlSwitchAction,
                      psa::ManualControlSwitchGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::ManualControlArmState, be::Land, psa::Landed,
                      psa::ManualControlSwitchAction,
                      psa::ManualControlSwitchGuard>> {};
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
    "ArmPreTakeoffFolding",
    "Takingoff",
    "Hovering",
    "ResetVisualServoing",
    "WaitingForPick",
    "PrePickState",
    "ArmPreLandingFolding",
    "ReachingGoal",
    "ExecutingVelocityGoal",
    "PickState",
    "Landing",
    "ReachingPostPickWaypoint",
    "PrePlaceState",
    "PlaceState",
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
