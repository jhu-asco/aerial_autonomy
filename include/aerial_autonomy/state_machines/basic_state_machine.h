#pragma once
/**
 * Basic State Machine that handles flying and going to a goal state
 *
 * States:
 *      Landed
 *      TakingOff
 *      Hovering
 *      ReachingGoal
 *      Landing
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
#include <aerial_autonomy/actions_guards/basic_states.h>

// Robot System used
#include <aerial_autonomy/robot_systems/uav_system.h>

namespace msmf = boost::msm::front;
namespace be = basic_events;

// Forward Declaration
struct LogicStateMachineFrontEnd;

/**
* @brief Backend for Logic State Machine.
*
* Used to forward arguments to constructor, and process events
*/
using LogicStateMachine =
    boost::msm::back::thread_safe_state_machine<LogicStateMachineFrontEnd>;

/**
* @brief front-end: define the FSM structure
*/
class LogicStateMachineFrontEnd
    : public msmf::state_machine_def<LogicStateMachineFrontEnd> {
  // Add friend classes that can use the robot system
  template <class EventT, class RobotSystemT, class LogicStateMachineT>
  friend class ActionFunctor;

  template <class EventT, class RobotSystemT, class LogicStateMachineT>
  friend class GuardFunctor;

  template <class RobotSystemT1, class LogicStateMachineT>
  friend class EventAgnosticActionFunctor;

  template <class RobotSystemT1, class LogicStateMachineT>
  friend class EventAgnosticGuardFunctor;

protected:
  /**
  * @brief robot system used by states to get sensor data and send commands
  */
  UAVSystem &robot_system_;

public:
  /**
  * @brief type index to store the event that did not trigger any transition
  */
  std::type_index no_transition_event_index_ = typeid(NULL);
  /**
  * @brief Action to take on entering state machine
  *
  * @tparam Event type of event causing state machine to enter
  * @tparam FSM Backend finite state machine type to trigger events
  */
  template <class Event, class FSM> void on_entry(Event const &, FSM &) {
    std::cout << "entering: UAV system" << std::endl;
  }
  /**
  * @brief Action to take on leaving state machine
  *
  * @tparam Event type of event causing state machine to enter
  * @tparam FSM Backend finite state machine type to trigger events
  */
  template <class Event, class FSM> void on_exit(Event const &, FSM &) {
    std::cout << "leaving: UAV system" << std::endl;
  }

  /**
  * @brief Constructor with arguments to store robot system
  *
  * @param uav_system robot system that is stored internally
  * and shared with events
  */
  LogicStateMachineFrontEnd(UAVSystem &uav_system)
      : robot_system_(uav_system) {}

  // States Used in the state machine:
  /**
  * @brief Takingoff state
  */
  using TakingOff = TakingOff_<LogicStateMachine>;
  /**
  * @brief Landing state
  */
  using Landing = Landing_<LogicStateMachine>;
  /**
  * @brief Reaching goal state
  */
  using ReachingGoal = ReachingGoal_<LogicStateMachine>;
  /**
  * @brief Hovering state
  */
  using Hovering = Hovering_<LogicStateMachine>;
  // States without any internal actions:
  /**
  * @brief Landed state
  */
  struct Landed : msmf::state<> {};
  /**
  * @brief Initial state for state machine
  */
  using initial_state = Landed;

  // Transition Actions
  /**
  * @brief Action to take when taking off
  */
  using TakeoffAction = TakeoffTransitionActionFunctor_<LogicStateMachine>;
  /**
  * @brief Guard to stop taking off under low voltage
  */
  using TakeoffGuard = TakeoffTransitionGuardFunctor_<LogicStateMachine>;
  /**
  * @brief Abort action when taking off
  */
  using TakeoffAbort = TakeoffAbortActionFunctor_<LogicStateMachine>;
  /**
  * @brief Action to take when landing
  */
  using LandingAction = LandTransitionActionFunctor_<LogicStateMachine>;
  /**
  * @brief set goal action when transitioning
  */
  using ReachingGoalSet =
      PositionControlTransitionActionFunctor_<LogicStateMachine>;
  /**
  * @brief Guard to avoid going to goal if goal is not correct
  */
  using ReachingGoalGuard =
      PositionControlTransitionGuardFunctor_<LogicStateMachine>;
  /**
  * @brief Abort action when reaching goal
  */
  using ReachingGoalAbort =
      PositionControlAbortActionFunctor_<LogicStateMachine>;
  /**
  * @brief Land action when reaching goal
  */
  using ReachingGoalLand = msmf::ActionSequence_<
      boost::mpl::vector<ReachingGoalAbort, LandingAction>>;

  /**
  * @brief Transition table for State Machine
  */
  struct transition_table
      : boost::mpl::vector<
            //        Start          Event         Next           Action Guard
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<Landed, be::Takeoff, TakingOff, TakeoffAction,
                      TakeoffGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<TakingOff, be::Land, Landing, LandingAction, msmf::none>,
            msmf::Row<TakingOff, be::Abort, Landing, TakeoffAbort, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<Hovering, PositionYaw, ReachingGoal, ReachingGoalSet,
                      ReachingGoalGuard>,
            msmf::Row<Hovering, be::Land, Landing, LandingAction, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<ReachingGoal, be::Abort, Hovering, ReachingGoalAbort,
                      msmf::none>,
            msmf::Row<ReachingGoal, be::Land, Landing, ReachingGoalLand,
                      msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<Landing, Completed, Landed, msmf::none, msmf::none>,
            msmf::Row<TakingOff, Completed, Hovering, msmf::none, msmf::none>,
            msmf::Row<ReachingGoal, Completed, Hovering, msmf::none, msmf::none>
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            > {};
  /**
  * @brief Print event typeid if no action present for the corresponding event
  *
  * @tparam FSM Backend to trigger events etc
  * @tparam Event Event type that triggered no transition
  * @param e event instance
  * @param state Current state when event is received
  */
  template <class FSM, class Event>
  void no_transition(Event const &e, FSM &, int state_index) {
    no_transition_event_index_ = typeid(e);
  }
};

/**
* @brief state names to get name based on state id
*/
static char const *const state_names[] = {"Landed", "TakingOff", "Hovering",
                                          "ReachingGoal", "Landing"};
/**
* @brief Get current state name
*
* @param p Logic state machine backend to access current state
*
* @return state name
*/
const char *pstate(LogicStateMachine const &p) {
  return state_names[p.current_state()[0]];
}
