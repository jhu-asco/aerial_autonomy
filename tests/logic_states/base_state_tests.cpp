#include <aerial_autonomy/logic_states/base_functors.h>
#include <aerial_autonomy/logic_states/base_state.h>
#include <gtest/gtest.h>

//// \brief Definitions
///  Define any necessary subclasses for tests here
struct RobotSystem {};
struct LogicStateMachine {};

struct EmptyRunFunctor : BaseRunFunctor<RobotSystem, LogicStateMachine> {
  virtual void run(RobotSystem const &, LogicStateMachine &) {}
};

struct EmptyGuardFunctor : BaseGuardFunctor<RobotSystem, LogicStateMachine> {
  virtual bool guard(RobotSystem const &, LogicStateMachine &) { return true; }
};
////

/// \brief TEST
/// All the tests are defined here
TEST(BaseStateTests, BaseStateCtor) {
  typedef BaseState<RobotSystem, LogicStateMachine, EmptyRunFunctor>
      BaseStateEmpty;
  ASSERT_NO_THROW(BaseStateEmpty());

  typedef BaseState<RobotSystem, LogicStateMachine, EmptyRunFunctor,
                    EmptyGuardFunctor>
      BaseStateEmptyGuard;
  ASSERT_NO_THROW(BaseStateEmptyGuard());
}

TEST(BaseStateTests, EmptytFctor) {
  EmptyRunFunctor empty_functor;
  RobotSystem robot_system;
  LogicStateMachine logic_state_machine;
  // Robotsystem, LogicStateMachine, source, target
  ASSERT_NO_THROW(empty_functor.run(robot_system, logic_state_machine));
}

TEST(BaseStateTests, EmptytGuardFctor) {
  EmptyGuardFunctor empty_functor;
  RobotSystem robot_system;
  LogicStateMachine logic_state_machine;
  // Robotsystem, LogicStateMachine, source, target
  ASSERT_TRUE(empty_functor.guard(robot_system, logic_state_machine));
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
