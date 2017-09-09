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
  // First run only updates the integrator but does not use it
  // in that step
  controller.run(sensor_data, controls);
  // Second run uses the integrated value in computing controls
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
  PositionYaw goal_diff = (goal - sensor_data);
  PositionYaw p_command = goal_diff * config.position_gain();
  controller.resetIntegrator();
  controller.setGoal(goal);
  VelocityYawRate controls;
  for (int i = 0; i < 1000; ++i) {
    controller.run(sensor_data, controls);
  }
  PositionYaw cumulative_error = controller.getCumulativeError();
  PositionYaw actual_input = p_command + cumulative_error;
  PositionYaw saturated_input = actual_input;
  saturated_input.clamp(
      PositionYaw(config.max_velocity(), config.max_yaw_rate()));
  PositionYaw diff_input = actual_input - saturated_input;
  double inv_gain_saturation = (1.0 / config.integrator_saturation_gain());
  ASSERT_NEAR(diff_input.x, inv_gain_saturation * goal_diff.x, 1e-3);
  ASSERT_NEAR(diff_input.y, inv_gain_saturation * goal_diff.y, 1e-3);
  ASSERT_NEAR(diff_input.z, inv_gain_saturation * goal_diff.z, 1e-3);
  ASSERT_NEAR(diff_input.yaw, inv_gain_saturation * goal_diff.yaw, 1e-3);
}

TEST(VelocityBasedPositionControllerTests, RunTillConvergenceWithBias) {
  double dt = 0.01;
  VelocityBasedPositionControllerConfig config;
  config.set_position_gain(2.0);
  config.set_yaw_gain(2.0);
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
