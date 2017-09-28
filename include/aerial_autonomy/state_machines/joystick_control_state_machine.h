#pragma once
/**
 * Basic State Machine that handles velocity commands from joystick
 *
 * Events:
 *      >>> Without args
 *      Land
 *      Takeoff
 *      Abort
 *      JoystickControlEvent
 */

// back-end
#include <aerial_autonomy/common/thread_safe_state_machine.h>

// front-end
#include <boost/msm/front/state_machine_def.hpp>

// functors
#include <boost/msm/front/functor_row.hpp>

// Actions and guards used
#include <aerial_autonomy/actions_guards/joystick_control_states_actions.h>

// Robot System used
#include <aerial_autonomy/robot_systems/uav_system.h>

// Logging library
#include <glog/logging.h>

// Store state machine states
#include <array>

// Base state machine
#include <aerial_autonomy/state_machines/base_state_machine.h>

// Events
#include <aerial_autonomy/joystick_control_events.h>

namespace msmf = boost::msm::front;
namespace jce = joystick_control_events;
namespace be = uav_basic_events;

// Forward Declaration
struct JoystickControlStateMachineFrontEnd;

/**
* @brief Backend for Logic State Machine.
*
* Used to forward arguments to constructor, and process events
*/
using JoystickControlStateMachine = boost::msm::back::thread_safe_state_machine<
    JoystickControlStateMachineFrontEnd>;

/**
* @brief Namespace for basic uav states and actions such as takeoff, land etc
*/
using jcsa = JoystickControlStatesActions<JoystickControlStateMachine>;

/**
* @brief front-end: define the FSM structure
*/
class JoystickControlStateMachineFrontEnd
    : public msmf::state_machine_def<JoystickControlStateMachineFrontEnd>,
      public BaseStateMachine<UAVSystem> {
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
  */
  JoystickControlStateMachineFrontEnd(UAVSystem &uav_system)
      : BaseStateMachine(uav_system) {}

  /**
  * @brief Initial state for state machine
  */
  using initial_state = jcsa::ManualControlState;
  /**
  * @brief Transition table for State Machine
  */
  struct transition_table
      : boost::mpl::vector<
            //        Start          Event         Next           Action Guard
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<jcsa::Landed, be::Takeoff, jcsa::TakingOff,
                      jcsa::TakeoffAction, jcsa::TakeoffGuard>,
            msmf::Row<jcsa::Landed, ManualControlEvent,
                      jcsa::ManualControlState, msmf::none, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<jcsa::TakingOff, Completed, jcsa::Hovering, msmf::none,
                      msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<jcsa::Hovering, be::Land, jcsa::Landing,
                      jcsa::LandingAction, msmf::none>,
            msmf::Row<jcsa::Hovering, jce::JoystickControlEvent,
                      jcsa::JoystickControlState, jcsa::JoystickControlAction,
                      jcsa::JoystickControlGuard>,
            msmf::Row<jcsa::Hovering, jce::SystemIdEvent, jcsa::SystemIdState,
                      jcsa::SystemIdStateAction, jcsa::SystemIdStateGuard>,
            msmf::Row<jcsa::Hovering, ManualControlEvent,
                      jcsa::ManualControlState, msmf::none, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<jcsa::Landing, Completed, jcsa::Landed, msmf::none,
                      msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<jcsa::JoystickControlState, be::Abort, jcsa::Hovering,
                      jcsa::UAVControllerAbort, msmf::none>,
            msmf::Row<jcsa::JoystickControlState, ManualControlEvent,
                      jcsa::ManualControlState, msmf::none, msmf::none>,
            msmf::Row<jcsa::JoystickControlState, jce::SystemIdEvent,
                      jcsa::SystemIdState, jcsa::SystemIdStateAction,
                      jcsa::SystemIdStateGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<jcsa::ManualControlState, be::Takeoff, jcsa::Hovering,
                      jcsa::ManualControlSwitchAction,
                      jcsa::ManualControlSwitchGuard>,
            msmf::Row<jcsa::ManualControlState, be::Land, jcsa::Landed,
                      jcsa::ManualControlSwitchAction,
                      jcsa::ManualControlSwitchGuard>,
            msmf::Row<jcsa::ManualControlState, jce::SystemIdEvent,
                      jcsa::SystemIdState, jcsa::SystemIdStateAction,
                      jcsa::SystemIdStateGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<jcsa::SystemIdState, be::Abort, jcsa::Hovering,
                      msmf::none, msmf::none>,
            msmf::Row<jcsa::SystemIdState, ManualControlEvent,
                      jcsa::ManualControlState, msmf::none, msmf::none>
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            > {};
  /**
  * @brief Use Inherited no transition function
  */
  using BaseStateMachine<UAVSystem>::no_transition;
};

/**
* @brief state names to get name based on state id
*/
static constexpr std::array<const char *, 7> state_names = {
    "Landed",
    "TakingOff",
    "Hovering",
    "Landing",
    "JoystickControlState",
    "ManualControlState",
    "SystemIdState"};
/**
* @brief Get current state name
*
* @param p Logic state machine backend to access current state
*
* @return state name
*/
const char *pstate(JoystickControlStateMachine const &p) {
  return state_names.at(p.current_state()[0]);
}
