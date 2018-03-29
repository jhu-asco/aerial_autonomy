#include <aerial_autonomy/state_machines/joystick_state_machine.h>
#include <aerial_autonomy/tests/test_utils.h>
#include <gtest/gtest.h>
// Thread stuff
#include <boost/optional/optional_io.hpp>
#include <boost/thread/thread.hpp>
// Quad Simulator
#include <quad_simulator_parser/quad_simulator.h>
#include <vector>

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
  std::unique_ptr<UAVRPYTStateMachine> logic_state_machine;
  std::unique_ptr<UAVSystem> uav_system;
  std::shared_ptr<QuadSimulator> drone_hardware;
  UAVSystemConfig config;
  BaseStateMachineConfig state_machine_config;

  virtual void SetUp() {
    drone_hardware.reset(new QuadSimulator);
    drone_hardware->usePerfectTime();
    auto position_tolerance = config.mutable_position_controller_config()
                                  ->mutable_goal_position_tolerance();
    position_tolerance->set_x(0.5);
    position_tolerance->set_y(0.5);
    position_tolerance->set_z(0.5);

    auto vel_based_pos_controller_config =
        config.mutable_rpyt_based_position_controller_config()
            ->mutable_velocity_based_position_controller_config();
    auto vel_based_pos_controller_tol =
        vel_based_pos_controller_config->mutable_position_controller_config()
            ->mutable_goal_position_tolerance();
    vel_based_pos_controller_tol->set_x(0.1);
    vel_based_pos_controller_tol->set_y(0.1);
    vel_based_pos_controller_tol->set_z(0.1);
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
    logic_state_machine.reset(new UAVRPYTStateMachine(
        boost::ref(*uav_system), boost::cref(state_machine_config)));
    logic_state_machine->start();
    // Will switch to Landed state from manual control state
    logic_state_machine->process_event(InternalTransitionEvent());

    LogConfig log_config;
    log_config.set_directory("/tmp/data");
    Log::instance().configure(log_config);
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

/// \brief Test Joystick Control related events
TEST_F(StateMachineTests, JoystickControl) {
  // First takeoff
  GoToHoverFromLanded();
  // Takeoff value is roughly 1400 so 2000 should make it go higher
  std::vector<int16_t> rc_values({0, 0, 2000, 0});
  // Set joystick value for uav
  drone_hardware->setRCInputsWithoutMapping(rc_values);
  // Start Joystick control
  logic_state_machine->process_event(be::Joystick());
  ASSERT_STREQ(pstate(*logic_state_machine), "RunningJoystickRPYTController");
  auto altitude_check = [&]() {
    uav_system->runActiveController(HardwareType::UAV);
    auto quad_data = uav_system->getUAVData();
    // Takeoff altitude is 2.0
    return (quad_data.localpos.z > 3.0);
  };
  ASSERT_TRUE(test_utils::waitUntilTrue()(
      altitude_check, std::chrono::seconds(1), std::chrono::milliseconds(0)));
}

TEST_F(StateMachineTests, JoystickControlAbort) {
  // First takeoff
  GoToHoverFromLanded();
  // Start Joystick control
  logic_state_machine->process_event(be::Joystick());
  ASSERT_STREQ(pstate(*logic_state_machine), "RunningJoystickRPYTController");
  // Abort goal
  logic_state_machine->process_event(be::Abort());
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
  // Check controller is actually aborted
  ASSERT_EQ(uav_system->getActiveControllerStatus(HardwareType::UAV),
            ControllerStatus::NotEngaged);
}

TEST_F(StateMachineTests, JoystickControlManualControlAbort) {
  // First takeoff
  GoToHoverFromLanded();
  // Start Joystick control
  logic_state_machine->process_event(be::Joystick());
  ASSERT_STREQ(pstate(*logic_state_machine), "RunningJoystickRPYTController");
  // Disable sdk
  drone_hardware->flowControl(false);
  // Run Internal Transition
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "ManualControlState");
  // Check controller is actually aborted
  ASSERT_EQ(uav_system->getActiveControllerStatus(HardwareType::UAV),
            ControllerStatus::NotEngaged);
}

TEST_F(StateMachineTests, JoystickControlLowBattery) {
  // First takeoff
  GoToHoverFromLanded();
  // Start Joystick control
  logic_state_machine->process_event(be::Joystick());
  ASSERT_STREQ(pstate(*logic_state_machine), "RunningJoystickRPYTController");
  // Low battery while reaching goal
  drone_hardware->setBatteryPercent(10);
  // Run Internal Transition
  logic_state_machine->process_event(InternalTransitionEvent());
  // Check if we are aborting due to low battery
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
  // Check controller is actually aborted
  ASSERT_EQ(uav_system->getActiveControllerStatus(HardwareType::UAV),
            ControllerStatus::NotEngaged);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
