#include "aerial_autonomy/controllers/joystick_velocity_controller.h"
#include "aerial_autonomy/tests/test_utils.h"
#include "joystick_velocity_controller_config.pb.h"
#include "rpyt_based_velocity_controller_config.pb.h"
#include <gtest/gtest.h>
#include <tf/tf.h>

TEST(JoystickVelocityControllerTests, ControlInBounds) {
  double dt = 0.02;
  RPYTBasedVelocityControllerConfig rpyt_config;
  JoystickVelocityControllerConfig joystick_config;
  JoystickVelocityController controller(rpyt_config, joystick_config, dt);
  controller.setGoal(EmptyGoal());

  Joystick joy_data(1000, -1000, 1000, 1000);
  VelocityYaw vel_data(0, 0, 0, 0);
  std::tuple<Joystick, VelocityYaw> sensor_data =
      std::make_tuple(joy_data, vel_data);
  double exp_yaw =
      math::angleWrap(vel_data.yaw - 0.1 * joystick_config.max_yaw_rate() * dt);

  RollPitchYawThrust controls;
  bool result = controller.run(sensor_data, controls);

  RollPitchYawThrust exp_controls;
  RPYTBasedVelocityControllerConfig config;
  RPYTBasedVelocityController internal_controller(config, dt);
  VelocityYaw vel_goal(0.1, -0.1, 0.1, exp_yaw);
  internal_controller.setGoal(vel_goal);
  internal_controller.run(vel_data, exp_controls);

  ASSERT_NEAR(controls.r, exp_controls.r, 1e-8);
  ASSERT_NEAR(controls.p, exp_controls.p, 1e-8);
  ASSERT_NEAR(controls.y, exp_controls.y, 1e-8);
  ASSERT_NEAR(controls.t, exp_controls.t, 1e-8);

  ASSERT_TRUE(result);
}

TEST(JoystickVelocityControllerTests, ControlOutOfBounds) {
  double dt = 0.02;
  RPYTBasedVelocityControllerConfig rpyt_config;
  JoystickVelocityControllerConfig joystick_config;
  JoystickVelocityController controller(rpyt_config, joystick_config, dt);
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
  double dt = 0.02;
  RPYTBasedVelocityControllerConfig rpyt_config;
  rpyt_config.set_kp(5.0);
  rpyt_config.set_ki(0.01);

  auto vel_ctlr_config = rpyt_config.mutable_velocity_controller_config();
  auto tolerance = vel_ctlr_config->mutable_goal_velocity_tolerance();
  tolerance->set_vx(0.01);
  tolerance->set_vy(0.01);
  tolerance->set_vz(0.01);

  JoystickVelocityControllerConfig joystick_config;
  JoystickVelocityController controller(rpyt_config, joystick_config, dt);
  Joystick joy_data(10000, 10000, 10000, 0);
  VelocityYaw vel_data(0, 0, 0, 0);
  std::tuple<Joystick, VelocityYaw> sensor_data =
      std::make_tuple(joy_data, vel_data);

  controller.setGoal(EmptyGoal());

  auto convergence = [&]() {

    RollPitchYawThrust controls;
    controller.run(sensor_data, controls);
    tf::Transform tf;
    tf.setOrigin(tf::Vector3(0, 0, 0));
    tf.setRotation(
        tf::createQuaternionFromRPY(controls.r, controls.p, controls.y));

    double ext_acc_z = 0.1;
    tf::Vector3 body_acc =
        tf::Vector3(0, 0, controls.t * rpyt_config.kt() + ext_acc_z);
    tf::Vector3 global_acc = tf * body_acc;

    VelocityYaw current_data = std::get<1>(sensor_data);
    VelocityYaw new_data;
    new_data.x = current_data.x + global_acc[0] * dt;
    new_data.y = current_data.y + global_acc[1] * dt;
    new_data.z = current_data.z + (global_acc[2] - 9.81) * dt;
    new_data.yaw = controls.y;

    std::get<1>(sensor_data) = new_data;

    return bool(controller.isConverged(sensor_data));
  };

  ASSERT_TRUE(
      test_utils::waitUntilTrue()(convergence, std::chrono::seconds(50)));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
