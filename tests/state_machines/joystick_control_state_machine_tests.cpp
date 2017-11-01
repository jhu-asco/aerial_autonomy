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
// Misc utitlities
#include <aerial_autonomy/common/math.h>
#include <aerial_autonomy/tests/test_utils.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>
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
  old_config.set_kt(0.2);
  uav_system->updateRPYTVelocityControllerConfig(old_config);
  // Takeoff
  GoToHoverFromLanded();

  ManualRPYTControllerConfig manual_rpyt_config =
      uav_system->getConfiguration().manual_rpyt_controller_config();
  // Go to SystemId state
  logic_state_machine->process_event(jce::SystemIdEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "SystemIdState");
  ASSERT_EQ(uav_system->getStatus<ManualRPYTControllerDroneConnector>(),
            ControllerStatus::Active);

  // Run Manual controller for ~20 seconds
  for (int i = 0; i < 1050; i++) {
    double t = math::map(i, 0, 1050, 0, 3.14);
    // Find RC thrust input s.t. z-component of acc is g
    double r =
        math::map(10000 * sin(t), -10000, 10000, -manual_rpyt_config.max_roll(),
                  manual_rpyt_config.max_roll());
    double p = math::map(10000 * sin(t), -10000, 10000,
                         -manual_rpyt_config.max_pitch(),
                         manual_rpyt_config.max_pitch());
    tf::Transform rot = tf::Transform(tf::createQuaternionFromRPY(r, p, 0),
                                      tf::Vector3(0, 0, 0));
    tf::Vector3 acc_dir = rot * tf::Vector3(0, 0, 1);
    double thrust = 9.81 / (0.16 * acc_dir[2]);
    double thrust_rc = math::map(thrust, 10, 100, -10000, 10000);

    int16_t channels[4] = {int16_t(10000 * sin(t)), int16_t(10000 * sin(t)),
                           int16_t(thrust_rc), int16_t(-10000 * sin(t))};
    drone_hardware.setRC(channels);
    uav_system->runActiveController(HardwareType::UAV);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    logic_state_machine->process_event(InternalTransitionEvent());
  }

  logic_state_machine->process_event(be::Abort());
  ASSERT_STREQ(pstate(*logic_state_machine), "Hovering");
  RPYTBasedVelocityControllerConfig new_config =
      uav_system->getRPYTVelocityControllerConfig();

  std::cout << "new Kt = " << new_config.kt() << std::endl;
  ASSERT_NEAR(new_config.kt(), 0.16, 0.005);

  // Run Velocity controller and check if z-velocity is
  // close to 0 when input is zero
  RPYTBasedVelocityControllerConfig newest_config;
  newest_config.set_kp(2.0);
  newest_config.set_ki(0.01);
  newest_config.set_kt(new_config.kt());
  uav_system->updateRPYTVelocityControllerConfig(newest_config);

  geometry_msgs::Vector3 vel;
  vel.x = 0.0;
  vel.y = 0.0;
  vel.z = 0.0;
  double yaw_rate = 0.0;
  drone_hardware.cmdvel_yaw_rate_guided(vel, yaw_rate);

  int16_t channels[4] = {0, 0, 0, 0};
  drone_hardware.setRC(channels);
  logic_state_machine->process_event(jce::JoystickControlEvent());
  ASSERT_STREQ(pstate(*logic_state_machine), "JoystickControlState");
  uav_system->runActiveController(HardwareType::UAV);

  // Run controller for ~10 seconds
  for (int j = 0; j < 500; j++) {
    uav_system->runActiveController(HardwareType::UAV);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  parsernode::common::quaddata data = uav_system->getUAVData();
  ASSERT_NEAR(data.linvel.z, 0.0, 0.05);
  ASSERT_NEAR(data.linvel.x, 0.0, 0.01);
  ASSERT_NEAR(data.linvel.y, 0.0, 0.01);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
