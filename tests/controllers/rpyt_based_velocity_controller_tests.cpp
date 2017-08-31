#include "aerial_autonomy/controllers/rpyt_based_velocity_controller.h"
#include <aerial_autonomy/tests/test_utils.h>
#include <chrono>
#include <gtest/gtest.h>
#include <tf/transform_datatypes.h>

TEST(RPYTBasedVelocityControllerTests, SetGetGoal) {
  RPYTBasedVelocityControllerConfig config;
  RPYTBasedVelocityController controller(config, 50);
  VelocityYaw sensor_data(0, 0, 0, 0);
  VelocityYaw goal(0.1, -0.1, 0.1, 0.1);
  controller.setGoal(goal);
  VelocityYaw exp_goal = controller.getGoal();
  ASSERT_EQ(exp_goal, goal);
}

TEST(RPYTBasedVelocityControllerTests, ControlsInBounds) {
  RPYTBasedVelocityControllerConfig config;
  RPYTBasedVelocityController controller(config, 50);
  VelocityYaw sensor_data(0, 0, 0, 0);
  VelocityYaw goal(0.1, -0.1, 0.1, 0.1);
  controller.setGoal(goal);
  VelocityYaw exp_goal = controller.getGoal();

  VelocityYaw velocity_diff = goal - sensor_data;
  RollPitchYawThrust controls;
  bool result = controller.run(sensor_data, controls);

  double dt = 0.02;
  double acc_x =
      config.kp() * velocity_diff.x + config.ki() * velocity_diff.x * dt;
  double acc_y =
      config.kp() * velocity_diff.y + config.ki() * velocity_diff.y * dt;
  double acc_z =
      config.kp() * velocity_diff.z + config.ki() * velocity_diff.z * dt + 9.81;

  double exp_t =
      sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z) / config.kt();

  double rot_acc_x =
      (acc_x * cos(sensor_data.yaw) + acc_y * sin(sensor_data.yaw)) /
      (config.kt() * controls.t);
  double rot_acc_y =
      (-acc_x * sin(sensor_data.yaw) + acc_y * cos(sensor_data.yaw)) /
      (config.kt() * controls.t);
  double rot_acc_z = acc_z / (config.kt() * controls.t);

  ASSERT_EQ(exp_goal, goal);
  ASSERT_NEAR(controls.y, goal.yaw, 1e-4);
  ASSERT_NEAR(controls.t, exp_t, 1e-4);
  ASSERT_NEAR(controls.r, -asin(rot_acc_y), 1e-4);
  ASSERT_NEAR(controls.p, atan2(rot_acc_x, rot_acc_z), 1e-4);

  ASSERT_TRUE(result);
}

TEST(RPYTBasedVelocityControllerTests, RollOutofBounds) {
  RPYTBasedVelocityControllerConfig config;
  RPYTBasedVelocityController controller(config, 50);
  VelocityYaw sensor_data(0, 0, 0, 0);
  VelocityYaw goal(0.0, 1.1, 0.0, 0.1);
  controller.setGoal(goal);
  RollPitchYawThrust controls;
  controller.run(sensor_data, controls);

  ASSERT_EQ(controls.p, 0);
}

TEST(RPYTBasedVelocityControllerTests, MaxThrust) {
  RPYTBasedVelocityControllerConfig config;
  RPYTBasedVelocityController controller(config, 50);
  VelocityYaw sensor_data(0, 0, 0, 0);
  VelocityYaw goal(10.0, 10.0, 10.0, 0.0);
  controller.setGoal(goal);
  RollPitchYawThrust controls;
  controller.run(sensor_data, controls);

  ASSERT_EQ(controls.t, config.max_thrust());
}

TEST(RPYTConvergenceTest, Convergence) {

  RPYTBasedVelocityControllerConfig config;
  config.set_kp(5.0);
  config.set_ki(0.01);

  auto vel_ctlr_config = config.mutable_velocity_controller_config();
  auto tolerance = vel_ctlr_config->mutable_goal_velocity_tolerance();
  tolerance->set_vx(0.01);
  tolerance->set_vy(0.01);
  tolerance->set_vz(0.01);

  RPYTBasedVelocityController controller(config, 50);

  VelocityYaw sensor_data;
  VelocityYaw goal(1.0, 1.0, 1.0, 0);
  controller.setGoal(goal);
  auto convergence = [&]() {

    RollPitchYawThrust controls;
    double dt = 0.02;

    controller.run(sensor_data, controls);
    tf::Transform tf;
    tf.setOrigin(tf::Vector3(0, 0, 0));
    tf.setRotation(
        tf::createQuaternionFromRPY(controls.r, controls.p, controls.y));

    double ext_z_acc = 0.1;
    tf::Vector3 body_acc =
        tf::Vector3(0, 0, controls.t * config.kt() + ext_z_acc);
    tf::Vector3 global_acc = tf * body_acc;

    sensor_data.x = sensor_data.x + global_acc[0] * dt;
    sensor_data.y = sensor_data.y + global_acc[1] * dt;
    sensor_data.z = sensor_data.z + (global_acc[2] - 9.81) * dt;
    sensor_data.yaw = controls.y;

    if (controller.isConverged(sensor_data))
      return true;

    return false;
  };

  ASSERT_TRUE(
      test_utils::waitUntilTrue()(convergence, std::chrono::seconds(50)));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
