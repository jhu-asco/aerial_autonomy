#include "joystick_velocity_controller_config.pb.h"
#include "rpyt_based_velocity_controller_config.pb.h"
#include <aerial_autonomy/controller_hardware_connectors/joystick_velocity_controller_drone_connector.h>
#include <aerial_autonomy/controllers/joystick_velocity_controller.h>
#include <aerial_autonomy/sensors/base_sensor.h>
#include <gtest/gtest.h>
#include <quad_simulator_parser/quad_simulator.h>
#include <thread>

using namespace quad_simulator;

TEST(JoystickVelocityControllerDroneConnectorTests, Constructor) {
  RPYTBasedVelocityControllerConfig rpyt_config;
  JoystickVelocityControllerConfig joystick_config;
  QuadSimulator drone_hardware;
  JoystickVelocityController controller(rpyt_config, joystick_config);
  Sensor<VelocityYaw> velocity_sensor;

  ASSERT_NO_THROW(new JoystickVelocityControllerDroneConnector(
    drone_hardware, controller, velocity_sensor));
}

TEST(JoystickVelocityControllerDroneConnectorTests, Run) {
  RPYTBasedVelocityControllerConfig rpyt_config;
  JoystickVelocityControllerConfig joystick_config;
  QuadSimulator drone_hardware;
  JoystickVelocityController controller(rpyt_config, joystick_config);
  Sensor<VelocityYaw> velocity_sensor;

  // Set stick commands
  int16_t channels[4] = {100, 100, 100, 0};
  drone_hardware.setRC(channels);

  JoystickVelocityControllerDroneConnector connector(drone_hardware, controller,
   velocity_sensor);

  connector.setGoal(EmptyGoal());

  for(int i=0; i<100; i++)
  {
    connector.run();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_EQ(connector.getStatus(), ControllerStatus::Completed);

  parsernode::common::quaddata sensor_data;
  drone_hardware.getquaddata(sensor_data);

  ASSERT_NEAR(sensor_data.servo_in[0], channels[0], 1e-4);
  ASSERT_NEAR(sensor_data.servo_in[1], channels[1], 1e-4);
  ASSERT_NEAR(sensor_data.servo_in[2], channels[2] , 1e-4);
  ASSERT_NEAR(sensor_data.servo_in[3], channels[3] , 1e-4);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}