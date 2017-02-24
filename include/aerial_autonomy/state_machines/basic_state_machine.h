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
#include <boost/msm/back/state_machine.hpp>

// front-end
#include <boost/msm/front/state_machine_def.hpp>

// functors
#include <boost/msm/front/functor_row.hpp>

// Actions and guards used
#include <aerial_autonomy/actions_guards/basic_states.h>

// Robot System used
#include <aerial_autonomy/robot_systems/uav_system.h>

namespace msmf = boost::msm::front;
using namespace basic_events;

// Forward Declaration
struct LogicStateMachineFrontEnd;

// Pick a back-end
using LogicStateMachine =
    boost::msm::back::state_machine<LogicStateMachineFrontEnd>;

// front-end: define the FSM structure
class LogicStateMachineFrontEnd
    : public msmf::state_machine_def<LogicStateMachineFrontEnd> {
  // Add friend classes that can use the robot system
  template <class EventT, class RobotSystemT, class LogicStateMachineT>
  friend class ActionFunctor;

  template <class EventT, class RobotSystemT, class LogicStateMachineT>
  friend class GuardFunctor;

protected:
  UAVSystem &robot_system_;

public:
  template <class Event, class FSM> void on_entry(Event const &, FSM &) {
    std::cout << "entering: UAV system" << std::endl;
  }
  template <class Event, class FSM> void on_exit(Event const &, FSM &) {
    std::cout << "leaving: UAV system" << std::endl;
  }

  // Constructor with arguments to store robot system
  LogicStateMachineFrontEnd(UAVSystem &uav_system)
      : robot_system_(uav_system) {}

  // States Used in the state machine:
  using TakingOff = TakingOff_<LogicStateMachine>;
  using Landing = Landing_<LogicStateMachine>;
  using ReachingGoal = ReachingGoal_<LogicStateMachine>;
  using Hovering = Hovering_<LogicStateMachine>;
  // States without any internal actions:
  struct Landed : msmf::state<> {};
  using initial_state = Landed;

  // Transition Actions
  using TakeoffAction = TakeoffTransitionActionFunctor_<LogicStateMachine>;
  using TakeoffGuard = TakeoffTransitionGuardFunctor_<LogicStateMachine>;
  using TakeoffAbort = TakeoffAbortActionFunctor_<LogicStateMachine>;
  using LandingAction = LandTransitionActionFunctor_<LogicStateMachine>;
  using ReachingGoalSet =
      PositionControlTransitionActionFunctor_<LogicStateMachine>;
  using ReachingGoalGuard =
      PositionControlTransitionGuardFunctor_<LogicStateMachine>;
  using ReachingGoalAbort =
      PositionControlAbortActionFunctor_<LogicStateMachine>;
  using ReachingGoalLand = msmf::ActionSequence_<
      boost::mpl::vector<ReachingGoalAbort, LandingAction>>;

  // Transition table for State Machine
  struct transition_table
      : boost::mpl::vector<
            //        Start          Event         Next           Action Guard
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<Landed, Takeoff, TakingOff, TakeoffAction, TakeoffGuard>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<TakingOff, Land, Landing, LandingAction, msmf::none>,
            msmf::Row<TakingOff, Abort, Landing, TakeoffAbort, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<Hovering, PositionYaw, ReachingGoal, ReachingGoalSet,
                      ReachingGoalGuard>,
            msmf::Row<Hovering, Land, Landing, LandingAction, msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<ReachingGoal, Abort, Hovering, ReachingGoalAbort,
                      msmf::none>,
            msmf::Row<ReachingGoal, Land, Landing, ReachingGoalLand,
                      msmf::none>,
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            msmf::Row<Landing, Completed, Landed, msmf::none, msmf::none>,
            msmf::Row<TakingOff, Completed, Hovering, msmf::none, msmf::none>,
            msmf::Row<ReachingGoal, Completed, Hovering, msmf::none, msmf::none>
            //        +--------------+-------------+--------------+---------------------+---------------------------+
            > {};
  // Replaces the default no-transition response.
  template <class FSM, class Event>
  void no_transition(Event const &e, FSM &, int state) {
    std::cout << "no transition from state " << state << " on event "
              << typeid(e).name() << std::endl;
  }
};

//
// Testing utilities.
//
static char const *const state_names[] = {"Landed", "TakingOff", "Hovering",
                                          "ReachingGoal", "Landing"};
void pstate(LogicStateMachine const &p) {
  std::cout << " -> " << state_names[p.current_state()[0]] << std::endl;
}
