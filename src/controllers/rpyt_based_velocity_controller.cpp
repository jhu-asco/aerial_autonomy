#include <aerial_autonomy/controllers/rpyt_based_velocity_controller.h>
#include <Eigen/Dense>

bool RPYTBasedVelocityController::runImplementation(VelocityYaw sensor_data,
  VelocityYaw goal, 
  RollPitchYawThrust &control)
{
  VelocityYaw velocity_diff = goal - sensor_data;

  cumulative_error.x += velocity_diff.x*config_.dt();
  cumulative_error.y += velocity_diff.y*config_.dt();
  cumulative_error.z += velocity_diff.z*config_.dt();

  // Acceleration in world frame 
  double acc_x = config_.kp()*velocity_diff.x + config_.ki()*cumulative_error.x;
  double acc_y = config_.kp()*velocity_diff.y + config_.ki()*cumulative_error.y;
  double acc_z = config_.kp()*velocity_diff.z + config_.ki()*cumulative_error.z;

  // Acceleration in body frame
  Eigen::Vector3d rot_acc;
  rot_acc[0] = acc_x*cos(sensor_data.yaw) + acc_y*sin(sensor_data.yaw);
  rot_acc[1] = -acc_x*sin(sensor_data.yaw) + acc_y*cos(sensor_data.yaw);
  rot_acc[2] = acc_z;

  // thrust is magnitude of scaled by kt
  control.t = rot_acc.norm()/config_.kt();

  // renormalize acceleration in body frame
  if(control.t > 1e-8)
    rot_acc = (1/(config_.kt()*control.t))*rot_acc;
  else
    rot_acc = Eigen::Vector3d(0,0,0);

  // roll = -asin(body_acc_y) when yaw-compensated
  control.r = -asin(rot_acc[1]);

  // check if roll is within tolerance and compute pitch accordingly
  if(rot_acc[1] < config_.tolerance_rp())
    control.p = atan2(rot_acc[0], rot_acc[2]);
  else
  {
    control.p = 0;
    std::cout << "RP out of bounds !\n";
  }

  control.y = goal.yaw;

  return true;
}

bool RPYTBasedVelocityController::isConvergedImplementation(
  VelocityYaw sensor_data, VelocityYaw goal)
{
  VelocityYaw velocity_diff = goal - sensor_data;
  const VelocityControllerConfig &velocity_controller_config =
  config_.velocity_controller_config();
  const double &tolerance_vel =
  velocity_controller_config.goal_velocity_tolerance();
  const double &tolerance_yaw = velocity_controller_config.goal_yaw_tolerance();
  // Compare
  if (std::abs(velocity_diff.x) < tolerance_vel &&
    std::abs(velocity_diff.y) < tolerance_vel &&
    std::abs(velocity_diff.z) < tolerance_vel &&
    std::abs(velocity_diff.yaw) < tolerance_yaw) {
    return true;
}
return false;
}
