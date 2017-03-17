#include <aerial_autonomy/state_machines/uav_state_machine.h>
#include <gtest/gtest.h>
// Thread stuff
#include <boost/optional/optional_io.hpp>
#include <boost/thread/thread.hpp>
// Quad Simulator
#include <quad_simulator_parser/quad_simulator.h>

/**
* @brief Namespace for UAV Simulator Hardware
*/
using namespace quad_simulator;

class StateMachineTests : public ::testing::Test {
protected:
  std::unique_ptr<UAVStateMachine> logic_state_machine;
  std::unique_ptr<UAVSystem> uav_system;
  QuadSimulator drone_hardware;

  virtual void SetUp() {
    drone_hardware.setTakeoffAltitude(2.0);
    uav_system.reset(new UAVSystem(drone_hardware));
    logic_state_machine.reset(new UAVStateMachine(boost::ref(*uav_system)));
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
    logic_state_machine->process_event(InternalTransitionEvent());
  }
};

TEST_F(StateMachineTests, InitialState) {
  // Validate
  ASSERT_STREQ(pstate(*logic_state_machine), "Landed");
}

TEST_F(StateMachineTests, NoTransitionEvent) {
  // We are in landed state, and we give Land command
  logic_state_machine->process_event(Land());
  ASSERT_EQ(logic_state_machine->get_no_transition_event_index(),
            std::type_index(typeid(Land)));
  // No Transition so we are still in landed state
  ASSERT_STREQ(pstate(*logic_state_machine), "Landed");
}

/// \brief Test Takeoff related events
TEST_F(StateMachineTests, LowBatteryTakeoff) {
  drone_hardware.setBatteryPercent(10);
  logic_state_machine->process_event(Takeoff());
  // Cannot takeoff
  ASSERT_STREQ(pstate(*logic_state_machine), "Landed");
}

TEST_F(StateMachineTests, LowBatteryAfterTakeoff) {
  drone_hardware.setBatteryPercent(100);
  logic_state_machine->process_event(Takeoff());
  ASSERT_STREQ(pstate(*logic_state_machine), "TakingOff");
  drone_hardware.setBatteryPercent(10);
  logic_state_machine->process_event(InternalTransitionEvent());
  // Cannot takeoff
  ASSERT_STREQ(pstate(*logic_state_machine), "Landing");
}

TEST_F(StateMachineTests, Takeoff) {
  drone_hardware.setBatteryPercent(100);
  logic_state_machine->process_event(Takeoff());
  // Takeoff in process
  ASSERT_STREQ(pstate(*logic_state_machine), "TakingOff");
  ASSERT_STREQ(uav_system->getUAVData().quadstate.c_str(),
               "ARMED ENABLE_CONTROL ");
  // Complete takeoff
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

TEST_F(StateMachineTests, PositionControlLowBattery) {
  // First takeoff
  GoToHoverFromLanded();
  // Now we are hovering, Lets go to a goal
  PositionYaw goal(0, 0, 5, 0);
  logic_state_machine->process_event(goal);
  ASSERT_STREQ(pstate(*logic_state_machine), "ReachingGoal");
  // Low battery while reaching goal
  drone_hardware.setBatteryPercent(10);
  // Run Internal Transition
  logic_state_machine->process_event(InternalTransitionEvent());
  // Check if we are landing due to low battery
  ASSERT_STREQ(pstate(*logic_state_machine), "Landing");
  // It should also abort the controller:
  uav_system->runActiveController(HardwareType::UAV);
  parsernode::common::quaddata data = uav_system->getUAVData();
  PositionYaw curr_pose_yaw(data.localpos.x, data.localpos.y, data.localpos.z,
                            data.rpydata.z);
  ASSERT_NE(curr_pose_yaw, goal);
}
///
/// \brief Test multi-thread access of logic state machine

class MultiThreadStateMachineTests : public StateMachineTests {
protected:
  boost::mutex signal_threads_mutex_;
  boost::condition_variable signal_condition_;

  void internalTransitionEventCall() {
    for (int count = 0; count < 200; ++count) {
      logic_state_machine->process_event(InternalTransitionEvent());
    }
  }

  void takeoffEventCall() {
    logic_state_machine->process_event(Takeoff());
    for (int count = 0; count < 200; ++count) {
      logic_state_machine->process_event(InternalTransitionEvent());
    }
  }
};

TEST_F(MultiThreadStateMachineTests, TakeoffMultiThread) {
  drone_hardware.setBatteryPercent(100);
  boost::thread t1(boost::bind(
      &MultiThreadStateMachineTests_TakeoffMultiThread_Test::takeoffEventCall,
      this));
  boost::thread t2(
      boost::bind(&MultiThreadStateMachineTests_TakeoffMultiThread_Test::
                      internalTransitionEventCall,
                  this));
  t1.join();
  t2.join();
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
  ASSERT_STREQ(uav_system->getUAVData().quadstate.c_str(),
               "ARMED ENABLE_CONTROL ");
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
