#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <aerial_autonomy/tests/sample_logic_state_machine.h>
#include <gtest/gtest.h>

//// \brief Definitions
///  Define any necessary subclasses for tests here
struct EmptyActionFunctor
    : EventAgnosticActionFunctor<EmptyRobotSystem, SampleLogicStateMachine> {
  virtual void run(EmptyRobotSystem &) {}
};

struct EmptyGuardFunctor
    : GuardFunctor<double, EmptyRobotSystem, SampleLogicStateMachine> {
  virtual bool guard(double const &, EmptyRobotSystem &) { return true; }
};
////

/// \brief TEST
/// All the tests are defined here
TEST(BaseStateTests, BaseStateCtor) {
  typedef BaseState<EmptyRobotSystem, SampleLogicStateMachine,
                    EmptyActionFunctor>
      BaseStateEmpty;
  ASSERT_NO_THROW(BaseStateEmpty());
}

TEST(BaseStateTests, EmptyFctor) {
  EmptyActionFunctor empty_functor;
  EmptyRobotSystem robot_system;
  SampleLogicStateMachine logic_state_machine(robot_system);
  // Robotsystem, SampleLogicStateMachine, source, target
  ASSERT_NO_THROW(empty_functor.run(robot_system));
  int empty_source_state, empty_target_state;
  ASSERT_NO_THROW(empty_functor(InternalTransitionEvent(), logic_state_machine,
                                empty_source_state, empty_target_state));
}

TEST(BaseStateTests, EmptyGuardFctor) {
  EmptyGuardFunctor empty_functor;
  EmptyRobotSystem robot_system;
  SampleLogicStateMachine logic_state_machine(robot_system);
  // Robotsystem, SampleLogicStateMachine, source, target
  ASSERT_TRUE(empty_functor.guard(double(0.0), robot_system));
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
