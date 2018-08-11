#pragma once
/**
 * VisualServoing State Machine that handles flying and going to a goal image
 *
 * Events:
 *      >>> Without args
 *      Land
 *      Takeoff
 *      Abort
 *      TrackROI
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
#include <aerial_autonomy/actions_guards/visual_servoing_states_actions.h>

// Robot System used
#include <aerial_autonomy/robot_systems/uav_vision_system.h>

// Logging library
#include <glog/logging.h>

// Store state machine states
#include <array>

// Base state machine
#include <aerial_autonomy/state_machines/base_state_machine.h>

// Events
#include <aerial_autonomy/visual_servoing_events.h>

#include <aerial_autonomy/types/manual_control_event.h>

namespace msmf = boost::msm::front;
namespace vse = visual_servoing_events;
namespace be = uav_basic_events;

// Forward Declaration
struct VisualServoingStateMachineFrontEnd;

/**
* @brief Backend for Logic State Machine.
*
* Used to forward arguments to constructor, and process events
*/
using VisualServoingStateMachine = boost::msm::back::thread_safe_state_machine<
    VisualServoingStateMachineFrontEnd>;

/**
* @brief Namespace for basic uav states and actions such as takeoff, land etc
*/
using vsa = VisualServoingStatesActions<VisualServoingStateMachine>;

/**
* @brief front-end: define the FSM structure
*/
class VisualServoingStateMachineFrontEnd
    : public msmf::state_machine_def<VisualServoingStateMachineFrontEnd>,
      public BaseStateMachine<UAVVisionSystem> {
public:
  /**
  * @brief Action to take on entering state machine
  *
  * @tparam Event type of event causing state machine to enter
  * @tparam FSM Backend finite state machine type to trigger events
  */
  template <class Event, class FSM> void on_entry(Event const &, FSM &) {
    VLOG(1) << "entering: VisualServoing system";
  }
  /**
  * @brief Action to take on leaving state machine
  *
  * @tparam Event type of event causing state machine to enter
  * @tparam FSM Backend finite state machine type to trigger events
  */
  template <class Event, class FSM> void on_exit(Event const &, FSM &) {
    VLOG(1) << "leaving: VisualServoing system";
  }

  /**
  * @brief Constructor with arguments to store robot system
  *
  * @param uav_system robot system that is stored internally
  * and shared with events
  * @param state_machine_config store config variables for state
  * machine
  */
  VisualServoingStateMachineFrontEnd(
      UAVVisionSystem &uav_system,
      const BaseStateMachineConfig &state_machine_config)
      : BaseStateMachine(uav_system, state_machine_config) {}

  /**
   * @brief Alternative constructor with default state machine config
   *
   * @param uav_system robot system that is stored internally
   * and shared with events
   */
  VisualServoingStateMachineFrontEnd(UAVVisionSystem &uav_system)
      : VisualServoingStateMachineFrontEnd(uav_system,
                                           BaseStateMachineConfig()){};

  /**
  * @brief Initial state for state machine
  */
  using initial_state = vsa::ManualControlState;
  /**
  * @brief Transition table for State Machine
  */
  struct transition_table
      : boost::mpl::vector<
            //        Start          Event         Next           Action Guard
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<vsa::Landed, be::Takeoff, vsa::TakingOff,
                      vsa::TakeoffAction, vsa::TakeoffGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<vsa::Landed, ManualControlEvent, vsa::ManualControlState,
                      msmf::none, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<vsa::TakingOff, Completed, vsa::Hovering, msmf::none,
                      msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<vsa::Hovering, PositionYaw, vsa::ReachingGoal,
                      vsa::ReachingGoalSet, vsa::ReachingGoalGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<vsa::Hovering, vse::TrackROI,
                      vsa::MPCRelativePoseVisualServoing,
                      vsa::MPCRelativePoseVisualServoingTransitionAction,
                      vsa::RelativePoseVisualServoingTransitionGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<vsa::Hovering, vse::GoHome, vsa::ReachingGoal,
                      vsa::GoHomeTransitionAction, vsa::GoHomeTransitionGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<vsa::Hovering, be::Land, vsa::Landing, vsa::LandingAction,
                      msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<vsa::Hovering, ManualControlEvent,
                      vsa::ManualControlState, msmf::none, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<vsa::ReachingGoal, be::Abort, vsa::Hovering,
                      vsa::UAVControllerAbort, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<vsa::ReachingGoal, be::Land, vsa::Landing,
                      vsa::ReachingGoalLand, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<vsa::MPCRelativePoseVisualServoing, be::Abort,
                      vsa::Hovering, vsa::UAVControllerAbort, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<vsa::Landing, Completed, vsa::Landed, msmf::none,
                      msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<vsa::ReachingGoal, Completed, vsa::Hovering,
                      vsa::UAVControllerAbort, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<vsa::MPCRelativePoseVisualServoing, Completed,
                      vsa::Hovering, vsa::UAVControllerAbort, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<vsa::ManualControlState, be::Takeoff, vsa::Hovering,
                      vsa::ManualControlSwitchAction,
                      vsa::ManualControlSwitchGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<vsa::ManualControlState, be::Land, vsa::Landed,
                      vsa::ManualControlSwitchAction,
                      vsa::ManualControlSwitchGuard>> {};
  /**
  * @brief Use Inherited no transition function
  */
  using BaseStateMachine<UAVVisionSystem>::no_transition;
};

/**
* @brief state names to get name based on state id
*/
static constexpr std::array<const char *, 7> state_names = {
    "Landed",         "TakingOff", "Hovering",          "ReachingGoal",
    "VisualServoing", "Landing",   "ManualControlState"};
/**
* @brief Get current state name
*
* @param p Logic state machine backend to access current state
*
* @return state name
*/
const char *pstate(VisualServoingStateMachine const &p) {
  return state_names.at(p.current_state()[0]);
}
