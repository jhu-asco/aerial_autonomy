#include "aerial_autonomy/controllers/rpyt_based_position_controller.h"
#include <gtest/gtest.h>

TEST(RPYTBasedPositionControllerTests, Constructor)
{
  ASSERT_NO_THROW(RPYTBasedPositionController());
}

TEST(RPYTBasedPositionControllerTests, ControlsInBound)
{
  RPYTBasedVelocityControllerConfig rpyt_vel_ctlr_config;
  VelocityBasedPositionControllerConfig vel_pos_ctrl_config;
  RPYTBasedPositionController controller(vel_pos_ctrl_config, rpyt_vel_ctlr_config);

  PositionYaw pos_data(0,0,0,0);
  VelocityYaw vel_data(0,0,0,0);

  std::tuple<PositionYaw, VelocityYaw> sensor_data;
  std::get<0>(sensor_data) = pos_data;
  std::get<1>(sensor_data) = vel_data;

  PositionYaw goal(1, -1, 0.5, 0.1);
  controller.setGoal(goal);

  RollPitchYawThrust controls;
  bool result = controller.run(sensor_data, controls);

  PositionYaw position_diff = goal - pos_data;
  VelocityYaw exp_vel(position_diff.x*vel_pos_ctrl_config.position_gain(),
    position_diff.y*vel_pos_ctrl_config.position_gain(),
    position_diff.z*vel_pos_ctrl_config.position_gain(),
    position_diff.yaw*vel_pos_ctrl_config.yaw_gain());

  VelocityYaw velocity_diff = exp_vel - vel_data;

  double dt = 0.03;
  double acc_x = rpyt_vel_ctlr_config.kp()*velocity_diff.x + rpyt_vel_ctlr_config.ki()*velocity_diff.x*dt;
  double acc_y = rpyt_vel_ctlr_config.kp()*velocity_diff.y + rpyt_vel_ctlr_config.ki()*velocity_diff.y*dt;
  double acc_z = rpyt_vel_ctlr_config.kp()*velocity_diff.z + rpyt_vel_ctlr_config.ki()*velocity_diff.z*dt;

  double exp_t = sqrt(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z)/rpyt_vel_ctlr_config.kt();

  double rot_acc_x = (acc_x*cos(vel_data.yaw) + acc_y*sin(vel_data.yaw))/(rpyt_vel_ctlr_config.kt()*controls.t);
  double rot_acc_y = (-acc_x*sin(vel_data.yaw) + acc_y*cos(vel_data.yaw))/(rpyt_vel_ctlr_config.kt()*controls.t);
  double rot_acc_z = acc_z/(rpyt_vel_ctlr_config.kt()*controls.t);
  
  ASSERT_NEAR(controls.t, exp_t, 1e-8);
  ASSERT_NEAR(controls.r, -asin(rot_acc_y),1e-8);
  ASSERT_NEAR(controls.p, atan2(rot_acc_x, rot_acc_z), 1e-8);
  ASSERT_NEAR(controls.y, exp_vel.yaw,1e-8);

  ASSERT_TRUE(result);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();  
}