#include <aerial_autonomy/joystick_control_events.h>
#include <aerial_autonomy/state_machines/joystick_control_state_machine.h>
#include <gtest/gtest.h>
// Thread stuff
#include <boost/optional/optional_io.hpp>
#include <boost/thread/thread.hpp>
// Quad Simulator
#include <quad_simulator_parser/quad_simulator.h>
// Guidance Sensor
#include <aerial_autonomy/sensors/base_sensor.h>
#include <aerial_autonomy/sensors/guidance.h>

/**
* @brief Namespace for UAV Simulator Hardware
*/
using namespace quad_simulator;

/**
* @brief  Namespace for joystick control events
*/
namespace jce = joystick_control_events;
namespace be = uav_basic_events;

class JoystickControlStateMachineTests : public ::testing::Test {
public:
  JoystickControlStateMachineTests() {
    rpyt_config.set_kp(5.0);
    rpyt_config.set_ki(0.01);

    auto vel_ctlr_config = rpyt_config.mutable_velocity_controller_config();
    auto tolerance = vel_ctlr_config->mutable_goal_velocity_tolerance();
    tolerance->set_vx(1e-4);
    tolerance->set_vy(1e-4);
    tolerance->set_vz(1e-4);

    drone_hardware.setTakeoffAltitude(2.0);

    velocity_sensor =
        std::shared_ptr<Sensor<std::tuple<VelocityYaw, Position>>>(
            new Guidance(drone_hardware));

    uav_system.reset(new UAVSystem(drone_hardware, uav_system_config,
                                   velocity_sensor, 0.02));
    logic_state_machine.reset(
        new JoystickControlStateMachine(boost::ref(*uav_system)));
    logic_state_machine->start();
    // Will switch to Landed state from manual control state
    logic_state_machine->process_event(InternalTransitionEvent());
  }

protected:
  std::unique_ptr<JoystickControlStateMachine> logic_state_machine;
  std::unique_ptr<UAVSystem> uav_system;
  UAVSystemConfig uav_system_config;
  QuadSimulator drone_hardware;
  std::shared_ptr<Sensor<std::tuple<VelocityYaw, Position>>> velocity_sensor;
  RPYTBasedVelocityControllerConfig rpyt_config;

  void GoToHoverFromLanded() {
    drone_hardware.setBatteryPercent(100);
    logic_state_machine->process_event(be::Takeoff());
    logic_state_machine->process_event(InternalTransitionEvent());
  }
};

TEST_F(JoystickControlStateMachineTests, InitialState) {
  ASSERT_STREQ(pstate(*logic_state_machine), "Landed");
}

TEST_F(JoystickControlStateMachineTests, GoToJoystickControl) {
  // First takeoff
  GoToHoverFromLanded();
  // Initialize Joystick Control
  logic_state_machine->process_event(jce::JoystickControlEvent());
  // Check controller status
  ASSERT_EQ(uav_system->getStatus<JoystickVelocityControllerDroneConnector>(),
            ControllerStatus::Active);
  // Check that current state is Joyctick control state
  ASSERT_STREQ(pstate(*logic_state_machine), "JoystickControlState");
}

TEST_F(JoystickControlStateMachineTests, Abort) {
  // First takeoff
  GoToHoverFromLanded();
  // Initialize Joystick Control
  logic_state_machine->process_event(jce::JoystickControlEvent());
  // Check that current state is Joyctick control state
  ASSERT_STREQ(pstate(*logic_state_machine), "JoystickControlState");
  logic_state_machine->process_event(be::Abort());
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
}

TEST_F(JoystickControlStateMachineTests, JoystickControlMode) {
  // First takeoff
  GoToHoverFromLanded();
  // Initialize Joystick Control
  logic_state_machine->process_event(jce::JoystickControlEvent());
  // Check that current state is Joyctick control state
  ASSERT_STREQ(pstate(*logic_state_machine), "JoystickControlState");
  // Disable SDK
  drone_hardware.flowControl(false);
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "ManualControlState");
  // Enable SDK
  drone_hardware.flowControl(true);
  logic_state_machine->process_event(InternalTransitionEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
}

TEST_F(JoystickControlStateMachineTests, SystemIdState) {
  // Set large kt
  RPYTBasedVelocityControllerConfig old_config;
  old_config.set_kt(0.5);
  uav_system->updateRPYTVelocityControllerConfig(old_config);
  // Takeoff
  GoToHoverFromLanded();
  // Go to SystemId state
  logic_state_machine->process_event(jce::SystemIdEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "SystemIdState");

  logic_state_machine->process_event(be::Abort());
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
