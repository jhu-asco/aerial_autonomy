#include "joystick_velocity_controller_config.pb.h"
#include "rpyt_based_velocity_controller_config.pb.h"
#include <aerial_autonomy/controller_hardware_connectors/joystick_velocity_controller_drone_connector.h>
#include <aerial_autonomy/controllers/joystick_velocity_controller.h>
#include <aerial_autonomy/sensors/base_sensor.h>
#include <gtest/gtest.h>
#include <quad_simulator_parser/quad_simulator.h>

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
  int16_t channels[4] = {0, 0, 0, 0};
  drone_hardware.setRC(channels);

  JoystickVelocityControllerDroneConnector connector(drone_hardware, controller,
                                                   velocity_sensor);

  connector.setGoal(EmptyGoal());
  connector.run();

  parsernode::common::quaddata quad_data;
  drone_hardware.getquaddata(quad_data);

  ASSERT_NEAR(quad_data.rpydata.x, 0.0, 1e-8);
  ASSERT_NEAR(quad_data.rpydata.y, 0.0, 1e-8);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}