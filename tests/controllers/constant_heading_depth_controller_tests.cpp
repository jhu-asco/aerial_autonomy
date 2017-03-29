#include "aerial_autonomy/controllers/constant_heading_depth_controller.h"

#include <gtest/gtest.h>

TEST(ConstantHeadingDepthControllerTests, Constructor) {
  ASSERT_NO_THROW(ConstantHeadingDepthController());
}

TEST(ConstantHeadingDepthControllerTests, Converged) {
  ConstantHeadingDepthControllerConfig config;
  ConstantHeadingDepthController controller(config);
  Position goal(1, .1, 0.5);
  controller.setGoal(goal);
  PositionYaw sensor_data(1, .1, 0.5, std::atan2(goal.y, goal.x));
  VelocityYawRate controls = controller.run(sensor_data);
  ASSERT_NEAR(controls.x, 0, 1e-10);
  ASSERT_NEAR(controls.y, 0, 1e-10);
  ASSERT_NEAR(controls.z, 0, 1e-10);
  ASSERT_NEAR(controls.yaw_rate, 0, 1e-10);
}

TEST(ConstantHeadingDepthControllerTests, NotConvergedRadial) {
  ConstantHeadingDepthControllerConfig config;
  config.set_tangential_gain(1);
  config.set_radial_gain(0.5);
  ConstantHeadingDepthController controller(config);
  Position goal(1, .1, 0.5);
  controller.setGoal(goal);
  PositionYaw sensor_data(2, .2, 1.0, std::atan2(goal.y, goal.x));
  VelocityYawRate controls = controller.run(sensor_data);
  ASSERT_NEAR(controls.x, 0.5, 1e-10);
  ASSERT_NEAR(controls.y, 0.05, 1e-10);
  ASSERT_NEAR(controls.z, 0.25, 1e-10);
  ASSERT_NEAR(controls.yaw_rate, 0, 1e-10);
}

TEST(ConstantHeadingDepthControllerTests, NotConvergedAll) {
  ConstantHeadingDepthControllerConfig config;
  config.set_tangential_gain(1);
  config.set_radial_gain(0.5);
  config.set_max_velocity(10);
  ConstantHeadingDepthController controller(config);
  Position goal(1, 0, 0);
  controller.setGoal(goal);
  PositionYaw sensor_data(2, 1, 0, std::atan2(goal.y, goal.x) + M_PI / 12);
  VelocityYawRate controls = controller.run(sensor_data);
  ASSERT_NEAR(controls.x, 0.5, 1e-10);
  ASSERT_NEAR(controls.y, 1., 1e-10);
  ASSERT_NEAR(controls.z, 0., 1e-10);
  ASSERT_NEAR(controls.yaw_rate, -config.yaw_gain() * M_PI / 12, 1e-10);
}

TEST(ConstantHeadingDepthControllerTests, NegativeNotConvergedAll) {
  ConstantHeadingDepthControllerConfig config;
  config.set_tangential_gain(1);
  config.set_radial_gain(0.5);
  config.set_max_velocity(10);
  ConstantHeadingDepthController controller(config);
  Position goal(-1, 0, 0);
  controller.setGoal(goal);
  PositionYaw sensor_data(-2, -1, 0, std::atan2(goal.y, goal.x) - M_PI / 12);
  VelocityYawRate controls = controller.run(sensor_data);
  ASSERT_NEAR(controls.x, -0.5, 1e-10);
  ASSERT_NEAR(controls.y, -1., 1e-10);
  ASSERT_NEAR(controls.z, 0., 1e-10);
  ASSERT_NEAR(controls.yaw_rate, config.yaw_gain() * M_PI / 12, 1e-10);
}

TEST(ConstantHeadingDepthControllerTests, MaxVelocity) {
  ConstantHeadingDepthControllerConfig config;
  config.set_tangential_gain(1);
  config.set_radial_gain(0.5);
  config.set_max_velocity(1);
  ConstantHeadingDepthController controller(config);
  Position goal(1, 0, 0);
  controller.setGoal(goal);
  PositionYaw sensor_data(2, 1, 0, std::atan2(goal.y, goal.x));
  VelocityYawRate controls = controller.run(sensor_data);
  ASSERT_NEAR(controls.x, 0.5 / std::sqrt(1.25), 1e-10);
  ASSERT_NEAR(controls.y, 1. / std::sqrt(1.25), 1e-10);
  ASSERT_NEAR(controls.z, 0., 1e-10);
  ASSERT_NEAR(controls.yaw_rate, 0, 1e-10);
}

TEST(ConstantHeadingDepthControllerTests, MaxYawRate) {
  ConstantHeadingDepthControllerConfig config;
  config.set_tangential_gain(1);
  config.set_radial_gain(0.5);
  config.set_max_velocity(10);
  config.set_max_yaw_rate(0.1);
  ConstantHeadingDepthController controller(config);
  Position goal(1, 1, 0);
  controller.setGoal(goal);
  PositionYaw sensor_data(2, 2, 1, std::atan2(goal.y, goal.x) - M_PI / 3);
  VelocityYawRate controls = controller.run(sensor_data);
  ASSERT_NEAR(controls.x, 0.5, 1e-10);
  ASSERT_NEAR(controls.y, 0.5, 1e-10);
  ASSERT_NEAR(controls.z, 1, 1e-10);
  ASSERT_NEAR(controls.yaw_rate, config.max_yaw_rate(), 1e-10);
}

TEST(ConstantHeadingDepthControllerTests, MaxNegativeYawRate) {
  ConstantHeadingDepthControllerConfig config;
  config.set_tangential_gain(1);
  config.set_radial_gain(0.5);
  config.set_max_velocity(10);
  config.set_max_yaw_rate(0.1);
  ConstantHeadingDepthController controller(config);
  Position goal(1, 1, 0);
  controller.setGoal(goal);
  PositionYaw sensor_data(2, 2, 1, std::atan2(goal.y, goal.x) + M_PI / 3);
  VelocityYawRate controls = controller.run(sensor_data);
  ASSERT_NEAR(controls.x, 0.5, 1e-10);
  ASSERT_NEAR(controls.y, 0.5, 1e-10);
  ASSERT_NEAR(controls.z, 1, 1e-10);
  ASSERT_NEAR(controls.yaw_rate, -config.max_yaw_rate(), 1e-10);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
