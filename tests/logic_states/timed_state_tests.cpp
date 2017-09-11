#include <aerial_autonomy/actions_guards/base_functors.h>
#include <aerial_autonomy/logic_states/timed_state.h>
#include <aerial_autonomy/tests/sample_logic_state_machine.h>
#include <gtest/gtest.h>

struct EmptyActionFunctor
    : EventAgnosticActionFunctor<EmptyRobotSystem, SampleLogicStateMachine> {
  virtual void run(EmptyRobotSystem &) {}
};

typedef TimedState<EmptyRobotSystem, SampleLogicStateMachine,
                   EmptyActionFunctor>
    TimedStateEmpty;

TEST(TimedStateTests, Constructor) { ASSERT_NO_THROW(TimedStateEmpty()); }

TEST(TmedStateTests, Timing) {
  TimedStateEmpty state;
  EmptyRobotSystem robot_system;
  SampleLogicStateMachine logic_state_machine(robot_system);
  state.on_entry(InternalTransitionEvent(), logic_state_machine);
  this_thread::sleep_for(std::chrono::milliseconds(200));
  ASSERT_GE(state.timeInState(), std::chrono::milliseconds(150));
  ASSERT_LE(state.timeInState(), std::chrono::milliseconds(250));
}

TEST(TmedStateTests, TimingReset) {
  TimedStateEmpty state;
  EmptyRobotSystem robot_system;
  SampleLogicStateMachine logic_state_machine(robot_system);
  state.on_entry(InternalTransitionEvent(), logic_state_machine);
  this_thread::sleep_for(std::chrono::milliseconds(200));
  state.on_entry(InternalTransitionEvent(), logic_state_machine);
  this_thread::sleep_for(std::chrono::milliseconds(200));
  ASSERT_GE(state.timeInState(), std::chrono::milliseconds(150));
  ASSERT_LE(state.timeInState(), std::chrono::milliseconds(250));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
