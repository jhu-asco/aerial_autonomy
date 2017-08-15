#include <aerial_autonomy/controllers/rpyt_based_velocity_controller.h>
#include <Eigen/Dense>

bool RPYTBasedVelocityController::runImplementation(VelocityYaw sensor_data,
  VelocityYaw goal, 
  RollPitchYawThrust &control)
{
  VelocityYaw velocity_diff = goal - sensor_data;

  cumulative_err.x += velocity_diff.x*dt;
  cumulative_err.y += velocity_diff.y*dt;
  cumulative_err.z += velocity_diff.z*dt;

  double acc_x = config_.kp()*velocity_diff.x + config_.ki()*cumulative_err.x;
  double acc_y = config_.kp()*velocity_diff.y + config_.ki()*cumulative_err.y;
  double acc_z = config_.kp()*velocity_diff.z + config_.ki()*cumulative_err.z;

  Eigen::Vector3d rot_acc;
  rot_acc[0] = acc_x*cos(sensor_data.yaw) + acc_y*sin(sensor_data.yaw);
  rot_acc[1] = -acc_x*sin(sensor_data.yaw) + acc_y*cos(sensor_data.yaw);
  rot_acc[2] = acc_z;

  control.t = rot_acc.norm()/config_.kt();

  rot_acc = (1/(config_.kt()*control.t))*rot_acc;

  control.r = -asin(rot_acc[1]);
  if(rot_acc[1] < config_.tolerance_rp())
    control.p = atan2(rot_acc[0], rot_acc[2]);
  else
  {
    control.p = 0;
    std::cout << "control out of bounds\n";
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
    //VLOG(1) << "Reached goal";
    return true;
  }
  return false;
  }
