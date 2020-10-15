#pragma once
/**
 * Orange Picking State Machine that handles flying, following a ROS path
 * and, (in the future) interacting with a target object.
 *
 * Events:
 *      >>> Without args
 *      Land
 *      Takeoff
 *      Abort
 *      Pick
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
//#include <aerial_autonomy/actions_guards/pick_place_states_actions.h>
#include <aerial_autonomy/actions_guards/orange_picking_states_actions.h>

// Robot System used
#include <aerial_autonomy/robot_systems/uav_arm_system.h>

// Logging library
#include <glog/logging.h>

// Store state machine states
#include <array>

// Base state machine
#include <aerial_autonomy/state_machines/base_state_machine.h>

// Events
//#include <aerial_autonomy/pick_place_events.h>
#include <aerial_autonomy/orange_picking_events.h>

#include <aerial_autonomy/types/manual_control_event.h>

namespace msmf = boost::msm::front;
namespace vse = visual_servoing_events;
namespace ope = orange_picking_events;
namespace be = uav_basic_events;

// Forward Declaration
struct OrangePickingStateMachineFrontEnd;

/**
* @brief Backend for Logic State Machine.
*
* Used to forward arguments to constructor, and process events
*/
using OrangePickingStateMachine =
    boost::msm::back::thread_safe_state_machine<OrangePickingStateMachineFrontEnd>;

/**
* @brief Namespace for basic uav states and actions such as takeoff, land etc
*/
using psa = PickPlaceStatesActions<OrangePickingStateMachine>;
using opsa = OrangePickingStatesActions<OrangePickingStateMachine>;

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
class OrangePickingStateMachineFrontEnd
    : public msmf::state_machine_def<OrangePickingStateMachineFrontEnd>,
      public BaseStateMachine<UAVArmSystem> {
public:
  /**
  * @brief Action to take on entering state machine
  *
  * @tparam Event type of event causing state machine to enter
  * @tparam FSM Backend finite state machine type to trigger events
  */
  template <class Event, class FSM> void on_entry(Event const &evt, FSM &fsm) {
    VLOG(1) << "entering: OrangePicking system";
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
    VLOG(1) << "leaving: OrangePicking system";
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
  OrangePickingStateMachineFrontEnd(
      UAVArmSystem &uav_system,
      const BaseStateMachineConfig &state_machine_config)
      : BaseStateMachine(uav_system, state_machine_config) {
    auto pick_state_machine_config =
        state_machine_config.visual_servoing_state_machine_config()
            .pick_place_state_machine_config();//TODO: set up config
    /*config_map_.insert<psa::ReachingPostPickWaypointBase>(
        pick_state_machine_config.following_waypoint_sequence_config());
    config_map_.insert<psa::ReachingPostPlaceWaypoint>(
        pick_state_machine_config.following_waypoint_sequence_config());
    config_map_.insert<psa::PickState>(pick_state_machine_config.grip_config());*/
  }

  /**
   * @brief Alternative constructor with default state machine config
   *
   * @param uav_system robot system that is stored internally and shared with
   * events
   */
  OrangePickingStateMachineFrontEnd(UAVArmSystem &uav_system)
      : OrangePickingStateMachineFrontEnd(uav_system, BaseStateMachineConfig()){};

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
            msmf::Row<psa::Hovering, be::Land, psa::ArmPreLandingFolding,
                      psa::ArmFold, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::Hovering, ManualControlEvent,
                      psa::ManualControlArmState, msmf::none, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::Hovering, PositionYaw, psa::ReachingGoal,
                      psa::ReachingGoalSet, psa::ReachingGoalGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::Hovering, vse::GoHome, psa::ReachingGoal,
                      psa::GoHomeTransitionAction, psa::GoHomeTransitionGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::Hovering, ope::Pick, opsa::PathFollowState, opsa::PathFollowTransitionAction,
                      opsa::PathFollowTransitionGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::ReachingGoal, be::Abort, psa::Hovering,
                      psa::AbortUAVControllerArmRightFold, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::ReachingGoal, Completed, psa::Hovering,
                      psa::AbortUAVControllerArmRightFold, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<opsa::PathFollowState, Completed, psa::Hovering,
                      psa::AbortUAVArmController, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<opsa::PathFollowState, be::Abort, psa::Hovering,
                      psa::AbortUAVArmController, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::ArmPreLandingFolding, Completed, psa::Landing,
                      psa::LandingAction, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::ArmPreLandingFolding, be::Abort, psa::Landing,
                      psa::LandingAction, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::Landing, Completed, psa::Landed, psa::ArmPowerOff,
                      msmf::none>,
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
    "ReachingGoal",
    "PathFollow",
    "ArmPreLandingFolding",
    "Landing",
    "ManualControlArmState"};
/**
* @brief Get current state name
*
* @param p Logic state machine backend to access current state
*
* @return state name
*/
const char *pstate(OrangePickingStateMachine const &p) {
  return state_names.at(p.current_state()[0]);
}
