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

TEST(RPYTControllerVINSConnectorTests, Run){
  QuadSimulator drone_hardware;
  RPYTBasedPositionController controller;

  // Set stick commands
  int16_t channels[4] = {0, 0, 0, 0};
  drone_hardware.setRC(channels);

  RPYTControllerVINSConnector connector(
    drone_hardware,
    controller);

  PositionYaw goal(0,0,0,0);
  connector.setGoal(goal);
  PositionYaw exp_goal = connector.getGoal();
  connector.run();

  parsernode::common::quaddata quad_data;
  drone_hardware.getquaddata(quad_data);

  ASSERT_EQ(goal, exp_goal);
  ASSERT_NEAR(quad_data.rpydata.x, 0.0, 1e-8);
  ASSERT_NEAR(quad_data.rpydata.y, 0.0, 1e-8);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}