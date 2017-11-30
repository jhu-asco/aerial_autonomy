#include "aerial_autonomy/controllers/rpyt_based_velocity_controller.h"
#include <aerial_autonomy/tests/test_utils.h>
#include <chrono>
#include <gtest/gtest.h>
#include <tf/transform_datatypes.h>

TEST(RPYTBasedVelocityControllerTests, SetGetGoal) {
  RPYTBasedVelocityControllerConfig config_;
  RPYTBasedVelocityController controller(config_, 0.02);
  VelocityYawRate goal(0.1, -0.1, 0.1, 0.1);
  controller.setGoal(goal);
  VelocityYawRate exp_goal = controller.getGoal();
  ASSERT_EQ(exp_goal, goal);
}

TEST(RPYTBasedVelocityControllerTests, ControlsInBounds) {
  double dt = 0.02;
  RPYTBasedVelocityControllerConfig config;
  RPYTBasedVelocityController controller(config, dt);
  auto sensor_data = std::make_tuple(VelocityYawRate(0, 0, 0, 0), 0.0);
  double yaw = std::get<1>(sensor_data);
  VelocityYawRate goal(0.1, -0.1, 0.1, 0.1);
  controller.setGoal(goal);

  VelocityYawRate velocity_diff = goal - std::get<0>(sensor_data);
  RollPitchYawRateThrust controls;
  bool result = controller.run(sensor_data, controls);

  double acc_x =
      config.kp_xy() * velocity_diff.x + config.ki_xy() * velocity_diff.x * dt;
  double acc_y =
      config.kp_xy() * velocity_diff.y + config.ki_xy() * velocity_diff.y * dt;
  double acc_z = config.kp_z() * velocity_diff.z +
                 config.ki_z() * velocity_diff.z * dt + 9.81;

  double exp_t =
      sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z) / config.kt();

  double rot_acc_x =
      (acc_x * cos(yaw) + acc_y * sin(yaw)) / (config.kt() * controls.t);
  double rot_acc_y =
      (-acc_x * sin(yaw) + acc_y * cos(yaw)) / (config.kt() * controls.t);
  double rot_acc_z = acc_z / (config.kt() * controls.t);

  ASSERT_NEAR(controls.y, goal.yaw_rate, 1e-4);
  ASSERT_NEAR(controls.t, exp_t, 1e-4);
  ASSERT_NEAR(controls.r, -asin(rot_acc_y), 1e-4);
  ASSERT_NEAR(controls.p, atan2(rot_acc_x, rot_acc_z), 1e-4);

  ASSERT_TRUE(result);
}

TEST(RPYTBasedVelocityControllerTests, ChangeConfig) {
  double dt = 0.02;
  RPYTBasedVelocityControllerConfig old_config;
  RPYTBasedVelocityController controller(old_config, dt);

  RPYTBasedVelocityControllerConfig config;
  config.set_kp_xy(5.0);
  config.set_kp_z(5.0);
  config.set_ki_xy(0.01);
  config.set_ki_z(0.01);
  config.set_kt(0.2);

  controller.updateConfig(config);

  auto sensor_data = std::make_tuple(VelocityYawRate(0, 0, 0, 0), 0.0);
  double yaw = std::get<1>(sensor_data);
  VelocityYawRate goal(0.1, -0.1, 0.1, 0.1);
  controller.setGoal(goal);

  VelocityYawRate velocity_diff = goal - std::get<0>(sensor_data);
  RollPitchYawRateThrust controls;
  bool result = controller.run(sensor_data, controls);

  double acc_x =
      config.kp_xy() * velocity_diff.x + config.ki_xy() * velocity_diff.x * dt;
  double acc_y =
      config.kp_xy() * velocity_diff.y + config.ki_xy() * velocity_diff.y * dt;
  double acc_z = config.kp_z() * velocity_diff.z +
                 config.ki_z() * velocity_diff.z * dt + 9.81;

  double exp_t =
      sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z) / config.kt();

  double rot_acc_x =
      (acc_x * cos(yaw) + acc_y * sin(yaw)) / (config.kt() * controls.t);
  double rot_acc_y =
      (-acc_x * sin(yaw) + acc_y * cos(yaw)) / (config.kt() * controls.t);
  double rot_acc_z = acc_z / (config.kt() * controls.t);

  ASSERT_NEAR(controls.y, goal.yaw_rate, 1e-4);
  ASSERT_NEAR(controls.t, exp_t, 1e-4);
  ASSERT_NEAR(controls.r, -asin(rot_acc_y), 1e-4);
  ASSERT_NEAR(controls.p, atan2(rot_acc_x, rot_acc_z), 1e-4);

  ASSERT_TRUE(result);
}

TEST(RPYTBasedVelocityControllerTests, RollNinety) {
  RPYTBasedVelocityControllerConfig config;
  config.set_kp_xy(1.0);
  config.set_kp_z(1.0);
  config.set_ki_xy(0.0);
  config.set_ki_z(0.0);

  RPYTBasedVelocityController controller(config, 0.02);
  auto sensor_data = std::make_tuple(VelocityYawRate(0, 0, 0, 0), 0.0);
  VelocityYawRate goal(0.0, 1.1, -9.81, 0.0);
  controller.setGoal(goal);
  RollPitchYawRateThrust controls;
  controller.run(sensor_data, controls);

  ASSERT_EQ(controls.p, 0);
}

TEST(RPYTBasedVelocityControllerTests, MaxThrust) {
  RPYTBasedVelocityControllerConfig config;
  RPYTBasedVelocityController controller(config, 0.02);
  auto sensor_data = std::make_tuple(VelocityYawRate(0, 0, 0, 0), 0.0);
  VelocityYawRate goal(10.0, 10.0, 10.0, 0.0);
  controller.setGoal(goal);
  RollPitchYawRateThrust controls;
  controller.run(sensor_data, controls);

  ASSERT_EQ(controls.t, config.max_thrust());
}

TEST(RPYTBasedVelocityControllerTests, MaxRoll) {

  RPYTBasedVelocityControllerConfig config;
  config.set_kp_xy(1.0);
  config.set_kp_z(1.0);
  config.set_ki_xy(0.0);
  config.set_ki_z(0.0);
  config.set_max_rp(1.0);

  RPYTBasedVelocityController controller(config, 0.02);
  auto sensor_data = std::make_tuple(VelocityYawRate(0, 0, 0, 0), 0.0);
  VelocityYawRate goal(0.0, 1.1, -9.81, 0.1);
  controller.setGoal(goal);
  RollPitchYawRateThrust controls;
  controller.run(sensor_data, controls);

  ASSERT_EQ(controls.r, -config.max_rp());
}

TEST(RPYTConvergenceTest, Convergence) {

  RPYTBasedVelocityControllerConfig config;
  config.set_kp_xy(5.0);
  config.set_kp_z(5.0);
  config.set_ki_xy(0.01);
  config.set_ki_z(0.01);

  auto vel_ctlr_config = config.mutable_velocity_controller_config();
  auto tolerance = vel_ctlr_config->mutable_goal_velocity_tolerance();
  tolerance->set_vx(0.01);
  tolerance->set_vy(0.01);
  tolerance->set_vz(0.01);

  double dt = 0.02;

  RPYTBasedVelocityController controller(config, dt);

  auto sensor_data = std::make_tuple(VelocityYawRate(0, 0, 0, 0), 0.0);
  auto &velocity_yaw_rate = std::get<0>(sensor_data);
  double &yaw = std::get<1>(sensor_data);
  VelocityYawRate goal(1.0, 1.0, 1.0, 0);
  controller.setGoal(goal);
  auto convergence = [&]() {

    RollPitchYawRateThrust controls;

    controller.run(sensor_data, controls);
    tf::Transform tf;
    tf.setOrigin(tf::Vector3(0, 0, 0));
    tf.setRotation(
        tf::createQuaternionFromRPY(controls.r, controls.p, controls.y));

    double ext_z_acc = 0.1;
    tf::Vector3 body_acc =
        tf::Vector3(0, 0, controls.t * config.kt() + ext_z_acc);
    tf::Vector3 global_acc = tf * body_acc;

    velocity_yaw_rate.x = velocity_yaw_rate.x + global_acc[0] * dt;
    velocity_yaw_rate.y = velocity_yaw_rate.y + global_acc[1] * dt;
    velocity_yaw_rate.z = velocity_yaw_rate.z + (global_acc[2] - 9.81) * dt;
    velocity_yaw_rate.yaw_rate = controls.y;
    yaw = yaw + controls.y * dt;

    return bool(controller.isConverged(sensor_data));

  };

  ASSERT_TRUE(test_utils::waitUntilTrue()(convergence, std::chrono::seconds(1),
                                          std::chrono::milliseconds(0)));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
