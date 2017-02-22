#include <aerial_autonomy/state_machines/basic_state_machine.h>
#include <aerial_autonomy/tests/sample_parser.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

/// \brief Test BuiltInPositionController
TEST(StateMachineTests, Constructor) {
  SampleParser drone_hardware;
  UAVSystem uav_system(drone_hardware);
  ASSERT_NO_THROW(new LogicStateMachine(boost::ref(uav_system)));
}

TEST(StateMachineTests, ProcessEvents) {
  SampleParser drone_hardware;
  UAVSystem uav_system(drone_hardware);
  LogicStateMachine logic_state_machine(boost::ref(uav_system));
  logic_state_machine.start();
  // Print initial state
  pstate(logic_state_machine);
  // Set right conditions
  logic_state_machine.process_event(Takeoff());
  // Check if we end up in right state
  // Print state
  pstate(logic_state_machine);
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
