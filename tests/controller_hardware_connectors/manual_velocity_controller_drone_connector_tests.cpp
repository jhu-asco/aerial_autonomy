#include <aerial_autonomy/controller_hardware_connectors/manual_velocity_controller_drone_connector.h>
#include <aerial_autonomy/controllers/manual_velocity_controller.h>
#include <gtest/gtest.h>
#include <quad_simulator_parser/quad_simulator.h>

using namespace quad_simulator;

TEST(ManualVelocityControllerDroneConnectorTests, Constructor)
{
  QuadSimulator drone_hardware;
  ManualVelocityController controller;

  ASSERT_NO_THROW(new ManualVelocityControllerDroneConnector(drone_hardware, controller));
}

int main(int argc, char **argv) {
  ros::init(argc,argv,"rpyt_connector_tets");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}