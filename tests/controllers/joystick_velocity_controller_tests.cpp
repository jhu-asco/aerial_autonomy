#include "aerial_autonomy/controllers/joystick_velocity_controller.h"
#include "joystick_velocity_controller_config.pb.h"
#include "rpyt_based_velocity_controller_config.pb.h"
#include <gtest/gtest.h>

TEST(JoystickVelocityControllerTests, ControlOutOfBounds){
  RPYTBasedVelocityControllerConfig rpyt_config;
  JoystickVelocityControllerConfig joystick_config;
  JoystickVelocityController controller(rpyt_config, joystick_config);
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

TEST(JoystickVelocityControllerTests, ControlInBounds){
  RPYTBasedVelocityControllerConfig rpyt_config;
  JoystickVelocityControllerConfig joystick_config;
  JoystickVelocityController controller(rpyt_config, joystick_config);
  EmptyGoal goal;
  controller.setGoal(goal);

  Joystick joy_data(5000, 5000, -5000, 1000);
  VelocityYaw vel_data(0.5, 0.5, -0.5, 0.1);
  std::tuple<Joystick, VelocityYaw> sensor_data =
  std::make_tuple(joy_data, vel_data);

  RollPitchYawThrust controls;
  bool result = controller.run(sensor_data, controls);

  double dt = 1.0/joystick_config.controller_frequency();
  double exp_yaw = vel_data.yaw - 0.1*dt;
  ASSERT_NEAR(controls.r, 0, 1e-4);
  ASSERT_NEAR(controls.p, 0, 1e-4);
  ASSERT_NEAR(controls.y, exp_yaw, 1e-4);
  ASSERT_NEAR(controls.t, 9.81 / rpyt_config.kt(), 1e-4);
  ASSERT_TRUE(result);
}


TEST(JoystickVelocityControllerTests, IsConvergedTest) {
  RPYTBasedVelocityControllerConfig rpyt_config;
  JoystickVelocityControllerConfig joystick_config;
  JoystickVelocityController controller(rpyt_config, joystick_config);
  EmptyGoal goal;
  controller.setGoal(goal);

  Joystick joy_data(0.0, 0.0, 0.0, 0.0);
  VelocityYaw vel_data(0, 0, 0, 0.0);
  std::tuple<Joystick, VelocityYaw> sensor_data =
  std::make_tuple(joy_data, vel_data);

  RollPitchYawThrust controls;
  bool result = controller.run(sensor_data, controls);

  ASSERT_NEAR(controls.y, 0, 1e-4);
  ASSERT_NEAR(controls.p, 0, 1e-4);
  ASSERT_NEAR(controls.t, 9.81 / rpyt_config.kt(), 1e-4);
  ASSERT_NEAR(controls.r, 0, 1e-4);
  ASSERT_TRUE(result);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}