#pragma once
/**
 * Basic State Machine that handles flying and going to a goal state
 *
 * Events:
 *      >>> Without args
 *      Land
 *      Takeoff
 *      Abort
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
#include <aerial_autonomy/actions_guards/uav_states_actions.h>
#include <aerial_autonomy/actions_guards/visual_servoing_states_actions.h>
#include <aerial_autonomy/visual_servoing_events.h>
// Robot System used
#include <aerial_autonomy/robot_systems/uav_vision_system.h>
// Logging library
#include <glog/logging.h>

// Store state machine states
#include <array>

// Base state machine
#include <aerial_autonomy/state_machines/base_state_machine.h>

namespace msmf = boost::msm::front;
namespace be = uav_basic_events;
namespace vse = visual_servoing_events;

// Forward Declaration
struct UAVStateMachineFrontEnd;

/**
* @brief Backend for Logic State Machine.
*
* Used to forward arguments to constructor, and process events
*/
using UAVStateMachine =
    boost::msm::back::thread_safe_state_machine<UAVStateMachineFrontEnd>;

/**
* @brief Namespace for basic uav states and actions such as takeoff, land etc
*/
using usa = UAVStatesActions<UAVStateMachine>;
using vsa = VisualServoingStatesActions<UAVStateMachine>;
/**
* @brief front-end: define the FSM structure
*/
class UAVStateMachineFrontEnd
    : public msmf::state_machine_def<UAVStateMachineFrontEnd>,
      public BaseStateMachine<UAVVisionSystem> {
public:
  /**
  * @brief Action to take on entering state machine
  *
  * @tparam Event type of event causing state machine to enter
  * @tparam FSM Backend finite state machine type to trigger events
  */
  template <class Event, class FSM> void on_entry(Event const &, FSM &) {
    VLOG(1) << "entering: UAV system";
  }
  /**
  * @brief Action to take on leaving state machine
  *
  * @tparam Event type of event causing state machine to enter
  * @tparam FSM Backend finite state machine type to trigger events
  */
  template <class Event, class FSM> void on_exit(Event const &, FSM &) {
    VLOG(1) << "leaving: UAV system";
  }

  /**
  * @brief Constructor with arguments to store robot system
  *
  * @param uav_system robot system that is stored internally
  * and shared with events
  * @param state_machine_config store config variables for state
  * machine
  */
  UAVStateMachineFrontEnd(UAVVisionSystem &uav_system,
                          const BaseStateMachineConfig &state_machine_config)
      : BaseStateMachine(uav_system, state_machine_config) {
    // Store state machine configs
  }

  /**
  * @brief Initial state for state machine
  */
  using initial_state = usa::ManualControlState;
  /**
  * @brief Transition table for State Machine
  */
  struct transition_table
      : boost::mpl::vector<
            //        Start          Event         Next           Action Guard
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<usa::Landed, be::Takeoff, usa::TakingOff,
                      usa::TakeoffAction, usa::TakeoffGuard>,
            msmf::Row<usa::Landed, ManualControlEvent, usa::ManualControlState,
                      msmf::none, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<usa::TakingOff, Completed, usa::Hovering, msmf::none,
                      msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<usa::Hovering, be::Search, usa::ReachingGoal1,
                      usa::ReachingGoalSet1, msmf::none>,
            msmf::Row<usa::Hovering, PositionYaw, usa::ReachingGoal,
                      usa::ReachingGoalSet, usa::ReachingGoalGuard>,
            msmf::Row<usa::Hovering, VelocityYaw, usa::ExecutingVelocityGoal,
                      usa::SetVelocityGoal, usa::GuardVelocityGoal>,
            msmf::Row<usa::Hovering, be::Land, usa::Landing, usa::LandingAction,
                      msmf::none>,
            msmf::Row<usa::Hovering, ManualControlEvent,
                      usa::ManualControlState, msmf::none, msmf::none>,
            msmf::Row<usa::Hovering, be::Joystick,
                      usa::RunningJoystickVelocityController,
                      usa::StartJoystickVelocityController, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<usa::ReachingGoal, be::Abort, usa::Hovering,
                      usa::UAVControllerAbort, msmf::none>,
            msmf::Row<usa::ReachingGoal, be::Land, usa::Landing,
                      usa::ReachingGoalLand, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<usa::ReachingGoal1, Completed, usa::ReachingGoal2,
                      usa::ReachingGoalSet2, msmf::none>,
            msmf::Row<usa::ReachingGoal1, vse::TrackROI , vsa::RelativePoseVisualServoing,
                      vsa::RelativePoseVisualServoingTransitionAction, vsa::RelativePoseVisualServoingTransitionGuard>,
            msmf::Row<usa::ReachingGoal2, Completed, usa::ReachingGoal3,
                      usa::ReachingGoalSet3, msmf::none>,
            msmf::Row<usa::ReachingGoal2, vse::TrackROI , vsa::RelativePoseVisualServoing,
                      vsa::RelativePoseVisualServoingTransitionAction, vsa::RelativePoseVisualServoingTransitionGuard>,
            msmf::Row<usa::ReachingGoal3, Completed, usa::ReachingGoal4,
                      usa::ReachingGoalSet4, msmf::none>,
            msmf::Row<usa::ReachingGoal3, vse::TrackROI , vsa::RelativePoseVisualServoing,
                      vsa::RelativePoseVisualServoingTransitionAction, vsa::RelativePoseVisualServoingTransitionGuard>,
            msmf::Row<usa::ReachingGoal4, Completed, usa::ReachingGoal1,
                      usa::ReachingGoalSet1, msmf::none>,
            msmf::Row<usa::ReachingGoal4, vse::TrackROI , vsa::RelativePoseVisualServoing,
                      vsa::RelativePoseVisualServoingTransitionAction, vsa::RelativePoseVisualServoingTransitionGuard>,

            msmf::Row<usa::ReachingGoal1, be::Abort, usa::Hovering,
                      usa::UAVControllerAbort, msmf::none>,
            msmf::Row<usa::ReachingGoal1, be::Land, usa::Landing,
                      usa::ReachingGoalLand, msmf::none>,
            msmf::Row<usa::ReachingGoal2, be::Abort, usa::Hovering,
                      usa::UAVControllerAbort, msmf::none>,
            msmf::Row<usa::ReachingGoal2, be::Land, usa::Landing,
                      usa::ReachingGoalLand, msmf::none>,
            msmf::Row<usa::ReachingGoal3, be::Abort, usa::Hovering,
                      usa::UAVControllerAbort, msmf::none>,
            msmf::Row<usa::ReachingGoal3, be::Land, usa::Landing,
                      usa::ReachingGoalLand, msmf::none>,
            msmf::Row<usa::ReachingGoal4, be::Abort, usa::Hovering,
                      usa::UAVControllerAbort, msmf::none>,
            msmf::Row<usa::ReachingGoal4, be::Land, usa::Landing,
                      usa::ReachingGoalLand, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<usa::ExecutingVelocityGoal, VelocityYaw,
                      usa::ExecutingVelocityGoal, usa::SetVelocityGoal,
                      usa::GuardVelocityGoal>,
            msmf::Row<usa::ExecutingVelocityGoal, be::Abort, usa::Hovering,
                      usa::UAVControllerAbort, msmf::none>,
            msmf::Row<usa::ExecutingVelocityGoal, be::Land, usa::Landing,
                      usa::ReachingGoalLand, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<usa::Landing, Completed, usa::Landed, msmf::none,
                      msmf::none>,
            msmf::Row<usa::ReachingGoal, Completed, usa::Hovering,
                      usa::UAVControllerAbort, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<usa::ManualControlState, be::Takeoff, usa::Hovering,
                      usa::ManualControlSwitchAction,
                      usa::ManualControlSwitchGuard>,
            msmf::Row<usa::ManualControlState, be::Land, usa::Landed,
                      usa::ManualControlSwitchAction,
                      usa::ManualControlSwitchGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<usa::RunningJoystickVelocityController, be::Abort,
                      usa::Hovering, usa::UAVControllerAbort, msmf::none>,
            msmf::Row<vsa::RelativePoseVisualServoing, be::Abort, usa::Hovering,
                      usa::UAVControllerAbort, msmf::none>> {};
  /**
  * @brief Use Inherited no transition function
  */
  using BaseStateMachine<UAVVisionSystem>::no_transition;
};

/**
* @brief state names to get name based on state id
*/
static constexpr std::array<const char *, 13> state_names = {
    "Landed",
    "TakingOff",
    "Hovering",
    "ReachingGoal",
    "ReachingGoal1",
    "ReachingGoal2",
    "ReachingGoal3",
    "ReachingGoal4",
    "ExecutingVelocityGoal",
    "Landing",
    "ManualControlState",
    "RunningJoystickVelocityController",
    "VisualServoing"};
/**
* @brief Get current state name
*
* @param p Logic state machine backend to access current state
*
* @return state name
*/
const char *pstate(UAVStateMachine const &p) {
  return state_names.at(p.current_state()[0]);
}
