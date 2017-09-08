#include "aerial_autonomy/controllers/velocity_based_position_controller.h"
#include <aerial_autonomy/tests/test_utils.h>

#include <gtest/gtest.h>

TEST(VelocityBasedPositionControllerTests, Constructor) {
  ASSERT_NO_THROW(VelocityBasedPositionController());
}

TEST(VelocityBasedPositionControllerTests, ControlsInBounds) {
  VelocityBasedPositionControllerConfig config;
  config.set_position_i_gain(0.0);
  config.set_yaw_i_gain(0.0);
  VelocityBasedPositionController controller(config);
  PositionYaw sensor_data(0, 0, 0, 0);
  PositionYaw goal(1, -1, 0.5, 0.1);
  controller.setGoal(goal);
  PositionYaw position_diff = goal - sensor_data;
  VelocityYawRate controls;
  bool result = controller.run(sensor_data, controls);
  ASSERT_NEAR(controls.x, position_diff.x * config.position_gain(), 1e-8);
  ASSERT_NEAR(controls.y, position_diff.y * config.position_gain(), 1e-8);
  ASSERT_NEAR(controls.z, position_diff.z * config.position_gain(), 1e-8);
  ASSERT_NEAR(controls.yaw_rate, position_diff.yaw * config.yaw_gain(), 1e-8);
  ASSERT_TRUE(result);
}

TEST(VelocityBasedPositionControllerTests, ControlsOutofBounds) {
  double dt = 0.02;
  VelocityBasedPositionControllerConfig config;
  VelocityBasedPositionController controller(config, dt);
  PositionYaw sensor_data(0, 0, 0, 0);
  PositionYaw goal(10, -10, 0.5, 1.5);
  controller.setGoal(goal);
  VelocityYawRate controls;
  double error_z = goal.z - sensor_data.z;
  double kp = config.position_gain();
  double ki = config.position_i_gain();
  double expected_command_z = error_z * kp + ki * dt * error_z;
  controller.run(sensor_data, controls);
  ASSERT_NEAR(controls.x, config.max_velocity(), 1e-8);
  ASSERT_NEAR(controls.y, -config.max_velocity(), 1e-8);
  ASSERT_NEAR(controls.z, expected_command_z, 1e-8);
  ASSERT_NEAR(controls.yaw_rate, config.max_yaw_rate(), 1e-8);
}

TEST(VelocityBasedPositionControllerTests, Converged) {
  VelocityBasedPositionControllerConfig config;
  auto tolerance = config.mutable_position_controller_config()
                       ->mutable_goal_position_tolerance();
  tolerance->set_x(0.1);
  tolerance->set_y(0.1);
  tolerance->set_z(0.1);

  VelocityBasedPositionController controller(config);
  PositionYaw sensor_data(0, 0, 0, 0);
  PositionYaw goal(0, 0, 0, 0);
  controller.setGoal(goal);
  VelocityYawRate controls;
  controller.run(sensor_data, controls);

  ASSERT_NEAR(controls.x, 0, 1e-6);
  ASSERT_NEAR(controls.y, 0, 1e-6);
  ASSERT_NEAR(controls.z, 0, 1e-6);
  ASSERT_NEAR(controls.yaw_rate, 0, 1e-6);
  ASSERT_TRUE(controller.isConverged(sensor_data));
}

TEST(VelocityBasedPositionControllerTests, ControlsOutofBoundsNegYaw) {
  VelocityBasedPositionControllerConfig config;
  VelocityBasedPositionController controller(config);
  PositionYaw sensor_data(0, 0, 0, 0);
  PositionYaw goal(-10, -10, 10, -1.5);
  controller.setGoal(goal);
  VelocityYawRate controls;
  controller.run(sensor_data, controls);
  ASSERT_NEAR(controls.x, -config.max_velocity(), 1e-8);
  ASSERT_NEAR(controls.y, -config.max_velocity(), 1e-8);
  ASSERT_NEAR(controls.z, config.max_velocity(), 1e-8);
  ASSERT_NEAR(controls.yaw_rate, -config.max_yaw_rate(), 1e-8);
}
TEST(VelocityBasedPositionControllerTests, WindingTest) {
  double dt = 0.01;
  VelocityBasedPositionControllerConfig config;
  config.set_max_velocity(0.2);
  config.set_max_yaw_rate(0.2);
  config.set_position_gain(1.0);
  config.set_yaw_gain(1.0);
  VelocityBasedPositionController controller(config, dt);
  PositionYaw sensor_data(0, 0, 0, 0);
  PositionYaw goal(5, -5, 5, 0.5);
  PositionYaw p_command = (goal - sensor_data) * config.position_gain();
  controller.resetIntegrator();
  controller.setGoal(goal);
  VelocityYawRate controls;
  controller.run(sensor_data, controls);
  PositionYaw cumulative_error = controller.getCumulativeError();
  ASSERT_NEAR(cumulative_error.x, config.max_velocity() - p_command.x, 1e-8);
  ASSERT_NEAR(cumulative_error.y, -1 * config.max_velocity() - p_command.y,
              1e-8);
  ASSERT_NEAR(cumulative_error.z, config.max_velocity() - p_command.z, 1e-8);
  ASSERT_NEAR(cumulative_error.yaw, config.max_yaw_rate() - p_command.yaw,
              1e-8);
}

TEST(VelocityBasedPositionControllerTests, RunTillConvergenceWithBias) {
  double dt = 0.01;
  VelocityBasedPositionControllerConfig config;
  config.set_position_gain(2.0);
  config.set_yaw_gain(2.0);
  config.set_position_i_gain(1.0);
  config.set_yaw_i_gain(1.0);
  config.mutable_position_controller_config()->set_goal_yaw_tolerance(0.05);
  auto position_tolerance = config.mutable_position_controller_config()
                                ->mutable_goal_position_tolerance();
  position_tolerance->set_x(0.05);
  position_tolerance->set_y(0.05);
  position_tolerance->set_z(0.05);
  VelocityBasedPositionController controller(config, dt);
  PositionYaw sensor_data(0, 0, 0, 0);
  PositionYaw goal(1, -1, 0.5, 0.5);
  controller.setGoal(goal);
  VelocityYawRate bias(0.1, 0.15, -0.15, 0.01);
  auto runController = [&]() {
    VelocityYawRate controls;
    controller.run(sensor_data, controls);
    sensor_data = sensor_data + (bias + controls) * dt;
    return bool(controller.isConverged(sensor_data));
  };
  ASSERT_TRUE(test_utils::waitUntilTrue()(
      runController, std::chrono::seconds(3), std::chrono::milliseconds(1)));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
