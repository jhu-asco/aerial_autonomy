#pragma once
/**
 * MPC State Machine  uses MPC controller to follow a reference trajectory
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
#include <aerial_autonomy/actions_guards/mpc_states_actions.h>

// Robot System used
#include <aerial_autonomy/robot_systems/uav_arm_system.h>

// Logging library
#include <glog/logging.h>

// Store state machine states
#include <array>

// Base state machine
#include <aerial_autonomy/state_machines/base_state_machine.h>

// Events
#include <aerial_autonomy/mpc_events.h>

#include <aerial_autonomy/types/manual_control_event.h>

namespace msmf = boost::msm::front;
namespace be = uav_basic_events;
namespace me = mpc_events;

// Forward Declaration
struct MPCStateMachineFrontEnd;

/**
* @brief Backend for Logic State Machine.
*
* Used to forward arguments to constructor, and process events
*/
using MPCStateMachine =
    boost::msm::back::thread_safe_state_machine<MPCStateMachineFrontEnd>;

/**
* @brief Namespace for states and actions used in state machine
*/
using msa = MPCStatesActions<MPCStateMachine>;

/**
* @brief front-end: define the FSM structure
*/
class MPCStateMachineFrontEnd
    : public msmf::state_machine_def<MPCStateMachineFrontEnd>,
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
  MPCStateMachineFrontEnd(UAVArmSystem &uav_arm_system,
                          const BaseStateMachineConfig &state_machine_config)
      : BaseStateMachine(uav_arm_system, state_machine_config) {}

  /**
  * @brief Initial state for state machine
  */
  using initial_state = msa::ManualControlArmState;
  /**
  * @brief Transition table for State Machine
  */
  struct transition_table
      : boost::mpl::vector<
            //        Start          Event         Next           Action Guard
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<msa::Landed, ManualControlEvent,
                      msa::ManualControlArmState, msmf::none, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<msa::Hovering, ManualControlEvent,
                      msa::ManualControlArmState, msmf::none, msmf::none>,
            msmf::Row<msa::Hovering, me::MPCSpiralTrack, msa::MPCState,
                      msa::MPCSpiralTransition, msmf::none>,
            msmf::Row<msa::Hovering, me::MPCWaypointTrack, msa::MPCState,
                      msa::MPCWaypointTransition, msmf::none>,
            msmf::Row<msa::Hovering, me::GoHome, msa::ReachingGoal,
                      msa::GoHomeTransitionAction, msa::GoHomeTransitionGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<msa::ReachingGoal, be::Abort, msa::Hovering,
                      msa::AbortUAVArmControllerArmRightFold, msmf::none>,
            msmf::Row<msa::ReachingGoal, Completed, msa::Hovering,
                      msa::AbortUAVArmControllerArmRightFold, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<msa::ManualControlArmState, be::Takeoff, msa::Hovering,
                      msa::ManualControlSwitchAction,
                      msa::ManualControlSwitchGuard>,
            msmf::Row<msa::ManualControlArmState, be::Land, msa::Landed,
                      msa::ManualControlSwitchAction,
                      msa::ManualControlSwitchGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<msa::MPCState, be::Abort, msa::Hovering,
                      msa::AbortUAVArmControllerArmRightFold, msmf::none>> {};
  /**
  * @brief Use Inherited no transition function
  */
  using BaseStateMachine<UAVArmSystem>::no_transition;
};

/**
* @brief state names to get name based on state id
*/
static constexpr std::array<const char *, 5> state_names = {
    "Landed", "Hovering", "ReachingGoal", "ManualControlArmState", "MPCState"};
/**
* @brief Get current state name
*
* @param p Logic state machine backend to access current state
*
* @return state name
*/
const char *pstate(MPCStateMachine const &p) {
  return state_names.at(p.current_state()[0]);
}
