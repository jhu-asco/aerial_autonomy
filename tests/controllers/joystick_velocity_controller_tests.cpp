#include "aerial_autonomy/controllers/joystick_velocity_controller.h"
#include "aerial_autonomy/tests/test_utils.h"
#include "joystick_velocity_controller_config.pb.h"
#include "rpyt_based_velocity_controller_config.pb.h"
#include <gtest/gtest.h>
#include <tf/tf.h>

TEST(JoystickVelocityControllerTests, ControlInBounds) {
  double timestep_ms = 20;
  RPYTBasedVelocityControllerConfig rpyt_config;
  JoystickVelocityControllerConfig joystick_config;
  JoystickVelocityController controller(rpyt_config, joystick_config,
                                        timestep_ms);
  controller.setGoal(EmptyGoal());

  Joystick joy_data(1000, -1000, 1000, 1000);
  VelocityYaw vel_data;
  std::tuple<Joystick, VelocityYaw> sensor_data =
      std::make_tuple(joy_data, vel_data);

  RollPitchYawThrust controls;
  bool result = controller.run(sensor_data, controls);

  VelocityYaw vel_goal(0.1, -0.1, 0.1, 0.1);
  VelocityYaw velocity_diff = vel_goal - vel_data;

  double dt = timestep_ms / 1000.0;
  double acc_x = rpyt_config.kp() * velocity_diff.x +
                 rpyt_config.ki() * velocity_diff.x * dt;
  double acc_y = rpyt_config.kp() * velocity_diff.y +
                 rpyt_config.ki() * velocity_diff.y * dt;
  double acc_z = rpyt_config.kp() * velocity_diff.z +
                 rpyt_config.ki() * velocity_diff.z * dt + 9.81;

  double exp_t =
      sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z) / rpyt_config.kt();

  double rot_acc_x = (acc_x * cos(vel_data.yaw) + acc_y * sin(vel_data.yaw)) /
                     (rpyt_config.kt() * controls.t);
  double rot_acc_y = (-acc_x * sin(vel_data.yaw) + acc_y * cos(vel_data.yaw)) /
                     (rpyt_config.kt() * controls.t);
  double rot_acc_z = acc_z / (rpyt_config.kt() * controls.t);

  double exp_yaw = vel_data.yaw - 0.1 * dt;

  ASSERT_NEAR(controls.y, exp_yaw, 1e-4);
  ASSERT_NEAR(controls.t, exp_t, 1e-4);
  ASSERT_NEAR(controls.r, -asin(rot_acc_y), 1e-4);
  ASSERT_NEAR(controls.p, atan2(rot_acc_x, rot_acc_z), 1e-4);
  ASSERT_TRUE(result);
}

TEST(JoystickVelocityControllerTests, ControlOutOfBounds) {
  double timestep_ms = 20;
  RPYTBasedVelocityControllerConfig rpyt_config;
  JoystickVelocityControllerConfig joystick_config;
  JoystickVelocityController controller(rpyt_config, joystick_config,
                                        timestep_ms);
  EmptyGoal goal;
  controller.setGoal(goal);

  Joystick joy_data(15000, 15000, -15000, 0);
  VelocityYaw vel_data(1, 1, -1, 0);
  std::tuple<Joystick, VelocityYaw> sensor_data =
      std::make_tuple(joy_data, vel_data);

  RollPitchYawThrust controls;
  bool result = controller.run(sensor_data, controls);

  ASSERT_NEAR(controls.r, 0, 1e-4);
  ASSERT_NEAR(controls.p, 0, 1e-4);
  ASSERT_NEAR(controls.y, 0, 1e-4);
  ASSERT_NEAR(controls.t, 9.81 / rpyt_config.kt(), 1e-4);
  ASSERT_TRUE(result);
}

TEST(JoystickVelocityControllerTests, Convergence) {
  double timestep_ms = 20.0;
  RPYTBasedVelocityControllerConfig rpyt_config;
  rpyt_config.set_kp(5.0);
  rpyt_config.set_ki(0.01);

  auto vel_ctlr_config = rpyt_config.mutable_velocity_controller_config();
  auto tolerance = vel_ctlr_config->mutable_goal_velocity_tolerance();
  tolerance->set_vx(0.01);
  tolerance->set_vy(0.01);
  tolerance->set_vz(0.01);

  JoystickVelocityControllerConfig joystick_config;
  JoystickVelocityController controller(rpyt_config, joystick_config,
                                        timestep_ms);
  Joystick joy_data(10000, 10000, -10000, 10000);
  VelocityYaw vel_data(0, 0, 0, 0);
  std::tuple<Joystick, VelocityYaw> sensor_data =
      std::make_tuple(joy_data, vel_data);

  controller.setGoal(EmptyGoal());

  auto convergence = [&]() {

    RollPitchYawThrust controls;
    controller.run(sensor_data, controls);

    controller.run(sensor_data, controls);
    tf::Transform tf;
    tf.setOrigin(tf::Vector3(0, 0, 0));
    tf.setRotation(
        tf::createQuaternionFromRPY(controls.r, controls.p, controls.y));

    tf::Vector3 body_acc = tf::Vector3(0, 0, controls.t * rpyt_config.kt());
    tf::Vector3 global_acc = tf * body_acc;

    double dt = timestep_ms / 1000.0;
    VelocityYaw current_data = std::get<1>(sensor_data);
    VelocityYaw new_data;
    new_data.x = current_data.x + global_acc[0] * dt;
    new_data.y = current_data.y + global_acc[1] * dt;
    new_data.z = current_data.z + (global_acc[2] - 9.81) * dt;
    new_data.yaw = controls.y;

    std::get<1>(sensor_data) = new_data;

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
