#pragma once
/**
 * Sensor Placement State Machine that handles flying, going to an object of
 * interest
 * and placing a payload there.
 *
 * Events:
 *      >>> Without args
 *      Land
 *      Takeoff
 *      Abort
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
#include <aerial_autonomy/actions_guards/sensor_place_states_actions.h>

// Robot System used
#include <aerial_autonomy/robot_systems/uav_arm_system.h>

// Logging library
#include <glog/logging.h>

// Store state machine states
#include <array>

// Base state machine
#include <aerial_autonomy/state_machines/base_state_machine.h>

// Events
#include <aerial_autonomy/sensor_place_events.h>

#include <aerial_autonomy/types/manual_control_event.h>

namespace msmf = boost::msm::front;
namespace vse = visual_servoing_events;
namespace se = sensor_place_events;
namespace be = uav_basic_events;

// Forward Declaration
struct SensorPlaceStateMachineFrontEnd;

/**
* @brief Backend for Logic State Machine.
*
* Used to forward arguments to constructor, and process events
*/
using SensorPlaceStateMachine = boost::msm::back::thread_safe_state_machine<
    SensorPlaceStateMachineFrontEnd>;

/**
* @brief Namespace for basic uav states and actions such as takeoff, land etc
*/
using psa = PickPlaceStatesActions<SensorPlaceStateMachine>;
using ssa = SensorPlaceStatesActions<SensorPlaceStateMachine>;

/**
* @brief front-end: define the FSM structure
*
* This state machine defines the logic for a simple
* sensor placement task.  After taking off, the robot is manually flown close
* to the placement goal.  After receiving an ROI, it will fly there and place
* the sensor.  After completion, it will retreat a safe distance and hover.
*/
class SensorPlaceStateMachineFrontEnd
    : public msmf::state_machine_def<SensorPlaceStateMachineFrontEnd>,
      public BaseStateMachine<UAVArmSystem> {
public:
  /**
  * @brief Action to take on entering state machine
  *
  * @tparam Event type of event causing state machine to enter
  * @tparam FSM Backend finite state machine type to trigger events
  */
  template <class Event, class FSM> void on_entry(Event const &evt, FSM &fsm) {
    VLOG(1) << "entering: Sensor Place system";
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
    VLOG(1) << "leaving: Sensor Place system";
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
  SensorPlaceStateMachineFrontEnd(
      UAVArmSystem &uav_system,
      const BaseStateMachineConfig &state_machine_config)
      : BaseStateMachine(uav_system, state_machine_config) {}

  /**
   * @brief Alternative constructor with default state machine config
   *
   * @param uav_system robot system that is stored internally and shared with
   * events
   */
  SensorPlaceStateMachineFrontEnd(UAVArmSystem &uav_system)
      : SensorPlaceStateMachineFrontEnd(uav_system, BaseStateMachineConfig()){};

  /**
  * @brief Initial state for state machine
  */
  using initial_state = psa::Landed;
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
            msmf::Row<psa::Landing, Completed, psa::Landed, psa::ArmPowerOff,
                      msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::ReachingGoal, Completed, psa::Hovering,
                      psa::AbortUAVControllerArmRightFold, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<psa::Hovering, se::Place, ssa::PreSensorPlaceState,
                      ssa::PreSensorPlaceVisualServoingTransitionAction,
                      ssa::PreSensorPlaceVisualServoingTransitionGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<ssa::PreSensorPlaceState, Completed,
                      ssa::SensorPlaceState,
                      ssa::SensorPlaceVisualServoingTransitionAction,
                      ssa::SensorPlaceVisualServoingTransitionGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<ssa::PreSensorPlaceState, be::Abort, psa::Hovering,
                      psa::AbortUAVArmController, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<ssa::SensorPlaceState, Completed,
                      ssa::SensorCheckingState,
                      ssa::SensorCheckingVisualServoingTransitionAction,
                      ssa::SensorCheckingVisualServoingTransitionGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<ssa::SensorPlaceState, Reset, ssa::PreSensorPlaceState,
                      ssa::PreSensorPlaceVisualServoingTransitionAction,
                      ssa::PreSensorPlaceVisualServoingTransitionGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<ssa::SensorPlaceState, be::Abort, psa::Hovering,
                      psa::AbortUAVArmController, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<ssa::SensorCheckingState, Completed, ssa::PostPlaceState,
                      ssa::PostPlaceVisualServoingTransitionAction,
                      ssa::PostPlaceVisualServoingTransitionGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<ssa::SensorCheckingState, Reset, ssa::PreSensorPlaceState,
                      ssa::PreSensorPlaceVisualServoingTransitionAction,
                      ssa::PreSensorPlaceVisualServoingTransitionGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<ssa::SensorCheckingState, be::Abort, psa::Hovering,
                      psa::AbortUAVArmController, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<ssa::PostPlaceState, Completed, psa::Hovering,
                      psa::AbortUAVArmController, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<ssa::PostPlaceState, be::Abort, psa::Hovering,
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
    "Landed",        "ArmPreTakeoffFolding", "Takingoff",
    "Hovering",      "ArmPreLandingFolding", "ReachingGoal",
    "Landing",       "PrePlaceState",        "PlaceState",
    "CheckingState", "PostPlaceState",       "ManualControlArmState"};
/**
* @brief Get current state name
*
* @param p Logic state machine backend to access current state
*
* @return state name
*/
const char *pstate(SensorPlaceStateMachine const &p) {
  return state_names.at(p.current_state()[0]);
}
