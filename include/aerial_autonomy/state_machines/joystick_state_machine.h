#pragma once
/**
 * Basic State Machine that handles flying and going to a goal state and
 * performs
 * rpyt flying using joystick
 *
 * Events:
 *      >>> Without args
 *      Land
 *      Takeoff
 *      Abort
 *      Joystick
 */

// back-end
#include <aerial_autonomy/common/thread_safe_state_machine.h>

// front-end
#include <boost/msm/front/state_machine_def.hpp>

// functors
#include <boost/msm/front/functor_row.hpp>

// Actions and guards used
#include <aerial_autonomy/actions_guards/uav_states_actions.h>

// Robot System used
#include <aerial_autonomy/robot_systems/uav_system.h>

// Logging library
#include <glog/logging.h>

// Store state machine states
#include <array>

// Base state machine
#include <aerial_autonomy/state_machines/base_state_machine.h>

namespace msmf = boost::msm::front;
namespace be = uav_basic_events;

// Forward Declaration
struct UAVRPYTStateMachineFrontEnd;

/**
* @brief Backend for Logic State Machine.
*
* Used to forward arguments to constructor, and process events
*/
using UAVRPYTStateMachine =
    boost::msm::back::thread_safe_state_machine<UAVRPYTStateMachineFrontEnd>;

/**
* @brief Namespace for basic uav states and actions such as takeoff, land etc
*/
using usa = UAVStatesActions<UAVRPYTStateMachine>;

/**
* @brief front-end: define the FSM structure
*/
class UAVRPYTStateMachineFrontEnd
    : public msmf::state_machine_def<UAVRPYTStateMachineFrontEnd>,
      public BaseStateMachine<UAVSystem> {
public:
  /**
  * @brief Constructor with arguments to store robot system
  *
  * @param uav_system robot system that is stored internally
  * and shared with events
  * @param state_machine_config store config variables for state
  * machine
  */
  UAVRPYTStateMachineFrontEnd(
      UAVSystem &uav_system, const BaseStateMachineConfig &state_machine_config)
      : BaseStateMachine(uav_system, state_machine_config) {}

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
            msmf::Row<usa::Hovering, be::Land, usa::Landing, usa::LandingAction,
                      msmf::none>,
            msmf::Row<usa::Hovering, ManualControlEvent,
                      usa::ManualControlState, msmf::none, msmf::none>,
            msmf::Row<usa::Hovering, be::Joystick,
                      usa::RunningJoystickRPYTController,
                      usa::StartJoystickRPYTController, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<usa::Landing, Completed, usa::Landed, msmf::none,
                      msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<usa::ManualControlState, be::Takeoff, usa::Hovering,
                      usa::ManualControlSwitchAction,
                      usa::ManualControlSwitchGuard>,
            msmf::Row<usa::ManualControlState, be::Land, usa::Landed,
                      usa::ManualControlSwitchAction,
                      usa::ManualControlSwitchGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<usa::RunningJoystickRPYTController, be::Abort,
                      usa::Hovering, usa::UAVControllerAbort, msmf::none>> {};
  /**
  * @brief Use Inherited no transition function
  */
  using BaseStateMachine<UAVSystem>::no_transition;
};

/**
* @brief state names to get name based on state id
*/
static constexpr std::array<const char *, 8> state_names = {
    "Landed",  "TakingOff",          "Hovering",
    "Landing", "ManualControlState", "RunningJoystickRPYTController"};
/**
* @brief Get current state name
*
* @param p Logic state machine backend to access current state
*
* @return state name
*/
const char *pstate(UAVRPYTStateMachine const &p) {
  return state_names.at(p.current_state()[0]);
}
