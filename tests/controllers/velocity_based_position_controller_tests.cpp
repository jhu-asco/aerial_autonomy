#include "aerial_autonomy/controllers/velocity_based_position_controller.h"

#include <gtest/gtest.h>

TEST(VelocityBasedPositionControllerTests, Constructor) {
  ASSERT_NO_THROW(VelocityBasedPositionController());
}

TEST(VelocityBasedPositionControllerTests, ControlsInBounds) {
  VelocityBasedPositionControllerConfig config;
  VelocityBasedPositionController controller(config);
  PositionYaw sensor_data(0, 0, 0, 0);
  PositionYaw goal(1, -1, 0.5, 0.1);
  controller.setGoal(goal);
  PositionYaw position_diff = goal - sensor_data;
  VelocityYaw controls;
  bool result = controller.run(sensor_data, controls);
  ASSERT_NEAR(controls.x, position_diff.x * config.position_gain(), 1e-8);
  ASSERT_NEAR(controls.y, position_diff.y * config.position_gain(), 1e-8);
  ASSERT_NEAR(controls.z, position_diff.z * config.position_gain(), 1e-8);
  ASSERT_NEAR(controls.yaw, position_diff.yaw * config.yaw_gain(), 1e-8);
  ASSERT_TRUE(result);
}

TEST(VelocityBasedPositionControllerTests, ControlsOutofBounds) {
  VelocityBasedPositionControllerConfig config;
  VelocityBasedPositionController controller(config);
  PositionYaw sensor_data(0, 0, 0, 0);
  PositionYaw goal(10, -10, 0.5, 1.5);
  controller.setGoal(goal);
  PositionYaw position_diff = goal - sensor_data;
  VelocityYaw controls;
  controller.run(sensor_data, controls);
  double position_norm = position_diff.norm();
  ASSERT_NEAR(controls.x,
              position_diff.x * config.max_velocity() / position_norm, 1e-8);
  ASSERT_NEAR(controls.y,
              position_diff.y * config.max_velocity() / position_norm, 1e-8);
  ASSERT_NEAR(controls.z,
              position_diff.z * config.max_velocity() / position_norm, 1e-8);
  ASSERT_NEAR(controls.yaw, config.max_yaw_rate(), 1e-8);
}

TEST(VelocityBasedPositionControllerTests, ControlsOutofBoundsNegYaw) {
  VelocityBasedPositionControllerConfig config;
  VelocityBasedPositionController controller(config);
  PositionYaw sensor_data(0, 0, 0, 0);
  PositionYaw goal(10, -10, 0.5, -1.5);
  controller.setGoal(goal);
  PositionYaw position_diff = goal - sensor_data;
  VelocityYaw controls;
  controller.run(sensor_data, controls);
  double position_norm = position_diff.norm();
  ASSERT_NEAR(controls.x,
              position_diff.x * config.max_velocity() / position_norm, 1e-8);
  ASSERT_NEAR(controls.y,
              position_diff.y * config.max_velocity() / position_norm, 1e-8);
  ASSERT_NEAR(controls.z,
              position_diff.z * config.max_velocity() / position_norm, 1e-8);
  ASSERT_NEAR(controls.yaw, -config.max_yaw_rate(), 1e-8);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
