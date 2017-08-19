#include <aerial_autonomy/controller_hardware_connectors/rpyt_controller_vins_connector.h>
#include <aerial_autonomy/controllers/rpyt_based_position_controller.h>
#include <gtest/gtest.h>
#include <quad_simulator_parser/quad_simulator.h>

using namespace quad_simulator;

TEST(RPYTControllerVINSConnectorTests, Constructor)
{
  QuadSimulator drone_hardware;
  RPYTBasedPositionController controller;

  ASSERT_NO_THROW(new RPYTControllerVINSConnector(drone_hardware, controller)); 
}

int main(int argc, char **argv) {
  ros::init(argc,argv,"rpyt_connector_tets");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}