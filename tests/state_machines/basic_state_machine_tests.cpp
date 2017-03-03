#include <aerial_autonomy/state_machines/basic_state_machine.h>
#include <aerial_autonomy/tests/sample_parser.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

class StateMachineTests : public ::testing::Test {
protected:
  std::unique_ptr<LogicStateMachine> logic_state_machine;
  std::unique_ptr<UAVSystem> uav_system;
  SampleParser drone_hardware;

  virtual void SetUp() {
    uav_system.reset(new UAVSystem(drone_hardware));
    logic_state_machine.reset(new LogicStateMachine(boost::ref(*uav_system)));
    logic_state_machine->start();
  }

  virtual void TearDown() {
    logic_state_machine->stop();
    uav_system.reset();
    logic_state_machine.reset();
  }

  void GoToHoverFromLanded() {
    drone_hardware.setBatteryPercent(100);
    logic_state_machine->process_event(Takeoff());
    drone_hardware.setaltitude(2.0);
    logic_state_machine->process_event(InternalTransitionEvent());
  }
};

TEST_F(StateMachineTests, InitialState) {
  // Validate
  ASSERT_STREQ(pstate(*logic_state_machine), "Landed");
}

/// \brief Test Takeoff related events
TEST_F(StateMachineTests, LowBatteryTakeoff) {
  drone_hardware.setBatteryPercent(10);
  logic_state_machine->process_event(Takeoff());
  // Cannot takeoff
  ASSERT_STREQ(pstate(*logic_state_machine), "Landed");
}

TEST_F(StateMachineTests, Takeoff) {
  drone_hardware.setBatteryPercent(100);
  logic_state_machine->process_event(Takeoff());
  // Takeoff in process
  ASSERT_STREQ(pstate(*logic_state_machine), "TakingOff");
  ASSERT_STREQ(uav_system->getUAVData().quadstate.c_str(), "takeoff");
  // Complete takeoff
  drone_hardware.setaltitude(2.0);
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
}
///
/// \brief Test Takeoff related events
TEST_F(StateMachineTests, Land) {
  // First takeoff
  GoToHoverFromLanded();
  // Now we are hovering, Lets Land!
  logic_state_machine->process_event(Land());
  ASSERT_STREQ(pstate(*logic_state_machine), "Landing");
  // Set altitude and check if landing is done
  drone_hardware.setaltitude(0.0);
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "Landed");
}
///

/// \brief Test Position Control related events
TEST_F(StateMachineTests, PositionControl) {
  // First takeoff
  GoToHoverFromLanded();
  // Now we are hovering, Lets go to a goal
  PositionYaw goal(0, 0, 5, 0);
  logic_state_machine->process_event(goal);
  ASSERT_STREQ(pstate(*logic_state_machine), "ReachingGoal");
  // Run active controller:
  uav_system->runActiveController(HardwareType::UAV);
  parsernode::common::quaddata data = uav_system->getUAVData();
  PositionYaw curr_pose_yaw(data.localpos.x, data.localpos.y, data.localpos.z,
                            data.rpydata.z);
  ASSERT_EQ(curr_pose_yaw, goal);
  // Verify if we reached the goal
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
}

TEST_F(StateMachineTests, PositionControlAbort) {
  // First takeoff
  GoToHoverFromLanded();
  // Now we are hovering, Lets go to a goal
  PositionYaw goal(0, 0, 5, 0);
  logic_state_machine->process_event(goal);
  // Abort goal
  logic_state_machine->process_event(Abort());
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
  // Once aborted running active controller will not reach goal
  uav_system->runActiveController(HardwareType::UAV);
  parsernode::common::quaddata data = uav_system->getUAVData();
  PositionYaw curr_pose_yaw(data.localpos.x, data.localpos.y, data.localpos.z,
                            data.rpydata.z);
  ASSERT_NE(curr_pose_yaw, goal);
}

TEST_F(StateMachineTests, PositionControlLand) {
  // First takeoff
  GoToHoverFromLanded();
  // Now we are hovering, Lets go to a goal
  PositionYaw goal(0, 0, 5, 0);
  logic_state_machine->process_event(goal);
  ASSERT_STREQ(pstate(*logic_state_machine), "ReachingGoal");
  // Land while reaching the goal
  logic_state_machine->process_event(Land());
  ASSERT_STREQ(pstate(*logic_state_machine), "Landing");
  // It should also abort the controller:
  uav_system->runActiveController(HardwareType::UAV);
  parsernode::common::quaddata data = uav_system->getUAVData();
  PositionYaw curr_pose_yaw(data.localpos.x, data.localpos.y, data.localpos.z,
                            data.rpydata.z);
  ASSERT_NE(curr_pose_yaw, goal);
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
