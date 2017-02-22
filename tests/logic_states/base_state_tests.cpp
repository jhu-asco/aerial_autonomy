#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <gtest/gtest.h>

//// \brief Definitions
///  Define any necessary subclasses for tests here
struct RobotSystem {
  void setGoal(){};
};
struct LogicStateMachine {
  LogicStateMachine(RobotSystem &robot_system) : robot_system_(robot_system) {}
  // Add friend classes:
  template <class EventT, class RobotSystemT, class LogicStateMachineT>
  friend class ActionFunctor;

  template <class EventT, class RobotSystemT, class LogicStateMachineT>
  friend class GuardFunctor;

protected:
  RobotSystem &robot_system_;
};

struct EmptyActionFunctor
    : InternalActionFunctor<RobotSystem, LogicStateMachine> {
  virtual void run(InternalTransitionEvent const &, RobotSystem &,
                   LogicStateMachine &) {}
};

struct EmptyGuardFunctor
    : GuardFunctor<double, RobotSystem, LogicStateMachine> {
  virtual bool guard(double const &, RobotSystem &, LogicStateMachine &) {
    return true;
  }
};
////

/// \brief TEST
/// All the tests are defined here
TEST(BaseStateTests, BaseStateCtor) {
  typedef BaseState<RobotSystem, LogicStateMachine, EmptyActionFunctor>
      BaseStateEmpty;
  ASSERT_NO_THROW(BaseStateEmpty());
}

TEST(BaseStateTests, EmptytFctor) {
  EmptyActionFunctor empty_functor;
  RobotSystem robot_system;
  LogicStateMachine logic_state_machine(robot_system);
  // Robotsystem, LogicStateMachine, source, target
  ASSERT_NO_THROW(empty_functor.run(InternalTransitionEvent(), robot_system,
                                    logic_state_machine));
  int empty_source_state, empty_target_state;
  ASSERT_NO_THROW(empty_functor(InternalTransitionEvent(), logic_state_machine,
                                empty_source_state, empty_target_state));
}

TEST(BaseStateTests, EmptytGuardFctor) {
  EmptyGuardFunctor empty_functor;
  RobotSystem robot_system;
  LogicStateMachine logic_state_machine(robot_system);
  // Robotsystem, LogicStateMachine, source, target
  ASSERT_TRUE(
      empty_functor.guard(double(0.0), robot_system, logic_state_machine));
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
