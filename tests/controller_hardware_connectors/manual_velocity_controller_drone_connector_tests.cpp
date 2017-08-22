#include <aerial_autonomy/controller_hardware_connectors/manual_velocity_controller_drone_connector.h>
#include <aerial_autonomy/controllers/manual_velocity_controller.h>
#include <gtest/gtest.h>
#include <quad_simulator_parser/quad_simulator.h>
#include <aerial_autonomy/sensors/base_sensor.h>

using namespace quad_simulator;

TEST(ManualVelocityControllerDroneConnectorTests, Constructor)
{
  QuadSimulator drone_hardware;
  ManualVelocityController controller;

  ASSERT_NO_THROW(new ManualVelocityControllerDroneConnector(
    drone_hardware, 
    controller));
}

TEST(ManualVelocityControllerDroneConnectorTests, Run){
  QuadSimulator drone_hardware;
  ManualVelocityController controller;

  // Set stick commands
  int16_t channels[4] = {0, 0, 0, 0};
  drone_hardware.setRC(channels);

  ManualVelocityControllerDroneConnector connector(
    drone_hardware,
    controller);

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