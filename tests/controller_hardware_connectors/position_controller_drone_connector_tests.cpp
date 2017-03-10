#include <aerial_autonomy/controller_hardware_connectors/position_controller_drone_connector.h>
#include <aerial_autonomy/controllers/builtin_controller.h>
#include <aerial_autonomy/types/position_yaw.h>
#include <gtest/gtest.h>
#include <quad_simulator_parser/quad_simulator.h>

/**
* @brief Namespace for UAV Simulator Hardware
*/
using namespace quad_simulator;

/// \brief Test BuiltInPositionController
TEST(PositionControllerDroneConnectorTests, Constructor) {
  QuadSimulator drone_hardware;

  BuiltInController<PositionYaw> position_controller;

  ASSERT_NO_THROW(new PositionControllerDroneConnector(drone_hardware,
                                                       position_controller));
}

TEST(PositionControllerDroneConnectorTests, SetGoal) {
  QuadSimulator drone_hardware;
  drone_hardware.takeoff();

  // Create controller and connector
  BuiltInController<PositionYaw> position_controller;
  PositionControllerDroneConnector position_controller_connector(
      drone_hardware, position_controller);

  // Test set goal
  PositionYaw goal(10, 10, 10, 0.1);
  position_controller_connector.setGoal(goal);
  PositionYaw goal_get = position_controller_connector.getGoal();
  ASSERT_EQ(goal_get.x, goal.x);
  ASSERT_EQ(goal_get.y, goal.y);
  ASSERT_EQ(goal_get.z, goal.z);
  position_controller_connector.run();

  parsernode::common::quaddata sensor_data;
  drone_hardware.getquaddata(sensor_data);

  ASSERT_EQ(sensor_data.localpos.x, goal.x);
  ASSERT_EQ(sensor_data.localpos.y, goal.y);
  ASSERT_EQ(sensor_data.localpos.z, goal.z);
  ASSERT_NEAR(sensor_data.rpydata.z, goal.yaw, 1e-8);
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
