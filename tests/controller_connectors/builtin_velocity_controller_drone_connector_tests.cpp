#include <aerial_autonomy/controller_connectors/builtin_velocity_controller_drone_connector.h>
#include <aerial_autonomy/controllers/builtin_controller.h>
#include <aerial_autonomy/types/velocity_yaw.h>
#include <gtest/gtest.h>
#include <quad_simulator_parser/quad_simulator.h>

/**
* @brief Namespace for UAV Simulator Hardware
*/
using namespace quad_simulator;

/// \brief Test BuiltInVelocityController
TEST(VelocityControllerDroneConnectorTests, Constructor) {
  QuadSimulator drone_hardware;

  BuiltInVelocityController velocity_controller;

  ASSERT_NO_THROW(new BuiltInVelocityControllerDroneConnector(
      drone_hardware, velocity_controller));
}

TEST(VelocityControllerDroneConnectorTests, SetGoal) {
  QuadSimulator drone_hardware;
  drone_hardware.takeoff();

  // Create controller and connector
  BuiltInVelocityController velocity_controller;
  BuiltInVelocityControllerDroneConnector velocity_controller_connector(
      drone_hardware, velocity_controller);

  // Test set goal
  VelocityYaw goal(10, 10, 10, 0.1);
  velocity_controller_connector.setGoal(goal);
  VelocityYaw goal_get = velocity_controller_connector.getGoal();
  ASSERT_EQ(goal_get.x, goal.x);
  ASSERT_EQ(goal_get.y, goal.y);
  ASSERT_EQ(goal_get.z, goal.z);
  velocity_controller_connector.run();

  parsernode::common::quaddata sensor_data;
  drone_hardware.getquaddata(sensor_data);

  ASSERT_EQ(sensor_data.linvel.x, goal.x);
  ASSERT_EQ(sensor_data.linvel.y, goal.y);
  ASSERT_EQ(sensor_data.linvel.z, goal.z);
  ASSERT_NEAR(sensor_data.rpydata.z, goal.yaw, 1e-8);
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
