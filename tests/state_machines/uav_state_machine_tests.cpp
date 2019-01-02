#include <aerial_autonomy/state_machines/uav_state_machine.h>
#include <aerial_autonomy/tests/test_utils.h>
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

/**
* @brief Namespace for basic events such as takeoff, land.
*/
namespace be = uav_basic_events;

class StateMachineTests : public ::testing::Test {
protected:
  std::unique_ptr<UAVStateMachine> logic_state_machine;
  std::unique_ptr<UAVSystem> uav_system;
  std::shared_ptr<QuadSimulator> drone_hardware;
  UAVSystemConfig config;
  BaseStateMachineConfig state_machine_config;

  virtual void SetUp() {
    drone_hardware.reset(new QuadSimulator);
    drone_hardware->usePerfectTime();
    config.mutable_rpyt_reference_connector_config()->set_use_perfect_time_diff(
        true);
    auto position_tolerance = config.mutable_position_controller_config()
                                  ->mutable_goal_position_tolerance();
    position_tolerance->set_x(0.5);
    position_tolerance->set_y(0.5);
    position_tolerance->set_z(0.5);

    auto vel_based_pos_controller_config =
        config.mutable_rpyt_based_position_controller_config()
            ->mutable_velocity_based_position_controller_config();
    vel_based_pos_controller_config->set_position_gain(20.);
    vel_based_pos_controller_config->set_yaw_gain(20.);
    auto vel_based_pos_controller_tol =
        vel_based_pos_controller_config->mutable_position_controller_config()
            ->mutable_goal_position_tolerance();
    vel_based_pos_controller_tol->set_x(0.1);
    vel_based_pos_controller_tol->set_y(0.1);
    vel_based_pos_controller_tol->set_z(0.1);
    vel_based_pos_controller_config->mutable_position_controller_config()
        ->set_goal_yaw_tolerance(0.1);
    auto rpyt_vel_controller_tol =
        config.mutable_rpyt_based_position_controller_config()
            ->mutable_rpyt_based_velocity_controller_config()
            ->mutable_velocity_controller_config()
            ->mutable_goal_velocity_tolerance();
    rpyt_vel_controller_tol->set_vx(0.1);
    rpyt_vel_controller_tol->set_vy(0.1);
    rpyt_vel_controller_tol->set_vz(0.1);

    drone_hardware->setTakeoffAltitude(2.0);
    uav_system.reset(new UAVSystem(
        config, std::dynamic_pointer_cast<parsernode::Parser>(drone_hardware)));
    logic_state_machine.reset(new UAVStateMachine(
        boost::ref(*uav_system), boost::cref(state_machine_config)));
    logic_state_machine->start();
    // Will switch to Landed state from manual control state
    logic_state_machine->process_event(InternalTransitionEvent());

    LogConfig log_config;
    log_config.set_directory("/tmp/data");
    Log::instance().configure(log_config);
    DataStreamConfig data_config;
    data_config.set_stream_id("velocity_based_position_controller");
    Log::instance().addDataStream(data_config);
    data_config.set_stream_id("rpyt_reference_connector");
    Log::instance().addDataStream(data_config);
    data_config.set_stream_id("rpyt_reference_controller");
    Log::instance().addDataStream(data_config);
    data_config.set_stream_id("thrust_gain_estimator");
    Log::instance().addDataStream(data_config);
    data_config.set_stream_id("visual_servoing_reference_connector");
    Log::instance().addDataStream(data_config);
  }

  virtual void TearDown() {
    logic_state_machine->stop();
    uav_system.reset();
    logic_state_machine.reset();
  }

  void GoToHoverFromLanded() {
    drone_hardware->setBatteryPercent(100);
    logic_state_machine->process_event(be::Takeoff());
    logic_state_machine->process_event(InternalTransitionEvent());
  }

  void runActiveControllerToConvergence() {
    auto getUAVStatusRunControllers = [&]() {
      uav_system->runActiveController(ControllerGroup::UAV);
      return uav_system->getActiveControllerStatus(ControllerGroup::UAV) ==
             ControllerStatus::Completed;
    };
    ASSERT_TRUE(test_utils::waitUntilTrue()(getUAVStatusRunControllers,
                                            std::chrono::seconds(5),
                                            std::chrono::milliseconds(0)));
  }
};

TEST_F(StateMachineTests, InitialState) {
  // Validate
  ASSERT_STREQ(pstate(*logic_state_machine), "Landed");
}

TEST_F(StateMachineTests, NoTransitionEvent) {
  // We are in landed state, and we give Land command
  logic_state_machine->process_event(be::Land());
  ASSERT_EQ(logic_state_machine->get_no_transition_event_index(),
            std::type_index(typeid(be::Land)));
  // No Transition so we are still in landed state
  ASSERT_STREQ(pstate(*logic_state_machine), "Landed");
}

/// \brief Test Takeoff related events
TEST_F(StateMachineTests, LowBatteryTakeoff) {
  drone_hardware->setBatteryPercent(10);
  logic_state_machine->process_event(be::Takeoff());
  // Cannot takeoff
  ASSERT_STREQ(pstate(*logic_state_machine), "Landed");
}

TEST_F(StateMachineTests, LowBatteryAfterTakeoff) {
  drone_hardware->setBatteryPercent(100);
  logic_state_machine->process_event(be::Takeoff());
  ASSERT_STREQ(pstate(*logic_state_machine), "TakingOff");
  drone_hardware->setBatteryPercent(10);
  logic_state_machine->process_event(InternalTransitionEvent());
  // We switch to Hovering here
  logic_state_machine->process_event(InternalTransitionEvent());
  // After Hovering if battery is low we stay in hovering
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
}

TEST_F(StateMachineTests, Takeoff) {
  drone_hardware->setBatteryPercent(100);
  logic_state_machine->process_event(be::Takeoff());
  // Takeoff in process
  ASSERT_STREQ(pstate(*logic_state_machine), "TakingOff");
  ASSERT_STREQ(uav_system->getUAVData().quadstate.c_str(),
               "ARMED ENABLE_CONTROL ");
  // Complete takeoff
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
}

TEST_F(StateMachineTests, TakeoffManualControlAbort) {
  drone_hardware->setBatteryPercent(100);
  // Takeoff
  logic_state_machine->process_event(be::Takeoff());
  // Disable sdk
  drone_hardware->flowControl(false);
  // While taking off we do not check for manual control event
  logic_state_machine->process_event(InternalTransitionEvent());
  // Check we are in Hovering
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
  // Since flow control is disabled, we should enter manual state
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "ManualControlState");
}
///
/// \brief Test Takeoff related events
TEST_F(StateMachineTests, Land) {
  // First takeoff
  GoToHoverFromLanded();
  // Now we are hovering, Lets Land!
  logic_state_machine->process_event(be::Land());
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
  // Send UAV to goal:
  runActiveControllerToConvergence();
  // Verify if we reached the goal
  uav_system->runActiveController(ControllerGroup::UAV); // Update status
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
}

// Check if position controller status is reset to active when setgoal is
// called
TEST_F(StateMachineTests, PositionControlResetActive) {
  // First takeoff
  GoToHoverFromLanded();
  // Now we are hovering, Lets go to a goal
  PositionYaw goal(0, 0, 5, 0);
  logic_state_machine->process_event(goal);
  // Run active controller:
  runActiveControllerToConvergence();
  // Transition to hovering
  logic_state_machine->process_event(InternalTransitionEvent());
  // Now try going to a second goal
  goal.x = 1.0;
  logic_state_machine->process_event(goal);
  // Check we are actually reaching goal
  ASSERT_STREQ(pstate(*logic_state_machine), "ReachingGoal");
  // Running internal transition now should not switch to
  // hovering since the controller status is active
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "ReachingGoal");
}

TEST_F(StateMachineTests, PositionControlAbort) {
  // First takeoff
  GoToHoverFromLanded();
  // Now we are hovering, Lets go to a goal
  PositionYaw goal(0, 0, 5, 0);
  logic_state_machine->process_event(goal);
  // Abort goal
  logic_state_machine->process_event(be::Abort());
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
  // Once aborted running active controller will not reach goal
  ASSERT_EQ(uav_system->getActiveControllerStatus(ControllerGroup::UAV),
            ControllerStatus::NotEngaged);
  parsernode::common::quaddata data = uav_system->getUAVData();
  PositionYaw curr_pose_yaw(data.localpos.x, data.localpos.y, data.localpos.z,
                            data.rpydata.z);
  ASSERT_NE(curr_pose_yaw, goal);
}

TEST_F(StateMachineTests, PositionControlManualControlAbort) {
  // First takeoff
  GoToHoverFromLanded();
  // Now we are hovering, Lets go to a goal
  PositionYaw goal(0, 0, 5, 0);
  logic_state_machine->process_event(goal);
  // Disable sdk
  drone_hardware->flowControl(false);
  // Run Internal Transition
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "ManualControlState");
  // Once aborted running active controller will not reach goal
  uav_system->runActiveController(ControllerGroup::UAV);
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
  logic_state_machine->process_event(be::Land());
  ASSERT_STREQ(pstate(*logic_state_machine), "Landing");
  // It should also abort the controller:
  uav_system->runActiveController(ControllerGroup::UAV);
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
  drone_hardware->setBatteryPercent(10);
  // Run Internal Transition
  logic_state_machine->process_event(InternalTransitionEvent());
  // Check if we are aborting due to low battery
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
  // It should also abort the controller:
  uav_system->runActiveController(ControllerGroup::UAV);
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
    logic_state_machine->process_event(be::Takeoff());
    for (int count = 0; count < 200; ++count) {
      logic_state_machine->process_event(InternalTransitionEvent());
    }
  }
};

TEST_F(MultiThreadStateMachineTests, TakeoffMultiThread) {
  drone_hardware->setBatteryPercent(100);
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
