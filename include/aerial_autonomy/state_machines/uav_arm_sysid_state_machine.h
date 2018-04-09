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

// Actions and guards used for uav
#include <aerial_autonomy/actions_guards/uav_arm_sysid_states_actions.h>

// Robot System used
#include <aerial_autonomy/robot_systems/uav_arm_system.h>

// Logging library
#include <glog/logging.h>

// Store state machine states
#include <array>

// Base state machine
#include <aerial_autonomy/state_machines/base_state_machine.h>

// Events
#include <aerial_autonomy/uav_arm_sysid_events.h>

#include <aerial_autonomy/types/manual_control_event.h>

namespace msmf = boost::msm::front;
namespace be = uav_basic_events;
namespace ue = uav_arm_sysid_events;

// Forward Declaration
struct UAVArmSysIDStateMachineFrontEnd;

/**
* @brief Backend for Logic State Machine.
*
* Used to forward arguments to constructor, and process events
*/
using UAVArmSysIDStateMachine = boost::msm::back::thread_safe_state_machine<
    UAVArmSysIDStateMachineFrontEnd>;

/**
* @brief Namespace for states and actions used in state machine
*/
using usa = UAVArmSysIDStatesActions<UAVArmSysIDStateMachine>;

/**
* @brief front-end: define the FSM structure
*/
class UAVArmSysIDStateMachineFrontEnd
    : public msmf::state_machine_def<UAVArmSysIDStateMachineFrontEnd>,
      public BaseStateMachine<UAVArmSystem> {
public:
  /**
  * @brief Constructor with arguments to store robot system
  *
  * @param uav_arm_system robot system that is stored internally
  * and shared with events
  * @param state_machine_config store config variables for state
  * machine
  */
  UAVArmSysIDStateMachineFrontEnd(
      UAVArmSystem &uav_arm_system,
      const BaseStateMachineConfig &state_machine_config)
      : BaseStateMachine(uav_arm_system, state_machine_config) {}

  /**
  * @brief Initial state for state machine
  */
  using initial_state = usa::ManualControlArmState;
  /**
  * @brief Transition table for State Machine
  */
  struct transition_table
      : boost::mpl::vector<
            //        Start          Event         Next           Action Guard
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<usa::Landed, ManualControlEvent,
                      usa::ManualControlArmState, msmf::none, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<usa::Hovering, ManualControlEvent,
                      usa::ManualControlArmState, msmf::none, msmf::none>,
            msmf::Row<usa::Hovering, ue::ArmOscillate,
                      usa::RunningArmSineController,
                      usa::ArmSineTransitionFunctor, msmf::none>,
            msmf::Row<usa::Hovering, be::Joystick,
                      usa::RunningJoystickRPYTController,
                      usa::StartJoystickRPYTController, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<usa::ManualControlArmState, be::Takeoff, usa::Hovering,
                      usa::ManualControlSwitchAction,
                      usa::ManualControlSwitchGuard>,
            msmf::Row<usa::ManualControlArmState, be::Land, usa::Landed,
                      usa::ManualControlSwitchAction,
                      usa::ManualControlSwitchGuard>,
            msmf::Row<usa::ManualControlArmState, ue::ArmOscillate,
                      usa::ArmStatus, usa::ArmSineTransitionFunctor,
                      msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<usa::ArmStatus, be::Abort, usa::Hovering,
                      usa::AbortArmControllerArmRightFold, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<usa::RunningArmSineController, be::Joystick,
                      usa::RunningJoystickRPYTArmSineController,
                      usa::StartJoystickRPYTController, msmf::none>,
            msmf::Row<usa::RunningArmSineController, be::Abort, usa::Hovering,
                      usa::AbortArmControllerArmRightFold, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<usa::RunningJoystickRPYTArmSineController, be::Abort,
                      usa::Hovering, usa::AbortUAVArmControllerArmRightFold,
                      msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<usa::RunningJoystickRPYTController, be::Abort,
                      usa::Hovering, usa::UAVControllerAbort, msmf::none>> {};
  /**
  * @brief Use Inherited no transition function
  */
  using BaseStateMachine<UAVArmSystem>::no_transition;
};

/**
* @brief state names to get name based on state id
*/
static constexpr std::array<const char *, 7> state_names = {
    "Landed",
    "Hovering",
    "ManualControlArmState",
    "ArmStatus",
    "RunningArmSineController",
    "RunningJoystickRPYTArmSineController",
    "RunningJoystickRPYTController"};
/**
* @brief Get current state name
*
* @param p Logic state machine backend to access current state
*
* @return state name
*/
const char *pstate(UAVArmSysIDStateMachine const &p) {
  return state_names.at(p.current_state()[0]);
}
