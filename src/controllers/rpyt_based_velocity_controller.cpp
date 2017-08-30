#include "aerial_autonomy/log/log.h"
#include <Eigen/Dense>
#include <aerial_autonomy/controllers/rpyt_based_velocity_controller.h>
#include <glog/logging.h>

bool RPYTBasedVelocityController::runImplementation(
    VelocityYaw sensor_data, VelocityYaw goal, RollPitchYawThrust &control) {
  VelocityYaw velocity_diff = goal - sensor_data;

  cumulative_error.x += velocity_diff.x * config_.dt();
  cumulative_error.y += velocity_diff.y * config_.dt();
  cumulative_error.z += velocity_diff.z * config_.dt();

  // Acceleration in world frame
  double acc_x =
      config_.kp() * velocity_diff.x + config_.ki() * cumulative_error.x;
  double acc_y =
      config_.kp() * velocity_diff.y + config_.ki() * cumulative_error.y;
  double acc_z =
      config_.kp() * velocity_diff.z + config_.ki() * cumulative_error.z + 9.81;

  // Acceleration in gravity aligned yaw-compensated frame
  Eigen::Vector3d rot_acc;
  rot_acc[0] = acc_x * cos(sensor_data.yaw) + acc_y * sin(sensor_data.yaw);
  rot_acc[1] = -acc_x * sin(sensor_data.yaw) + acc_y * cos(sensor_data.yaw);
  rot_acc[2] = acc_z;

  // thrust is magnitude of acceleration scaled by kt
  control.t = rot_acc.norm() / config_.kt();

  // normalize acceleration in gravity aligned yaw-compensated frame
  if (control.t > 1e-8)
    rot_acc = (1 / (config_.kt() * control.t)) * rot_acc;
  else {
    LOG(WARNING) << "Thrust close to zero !!";
    rot_acc = Eigen::Vector3d(0, 0, 0);
  }

  // yaw-compensated y-acceleration is sine of roll
  control.r = -asin(rot_acc[1]);

  // check if roll = 90 and compute pitch accordingly
  if ((abs(rot_acc[1]) - 1.0) < config_.tolerance_rp())
    control.p = atan2(rot_acc[0], rot_acc[2]);
  else {
    // if roll is 90, pitch is undefined and is set to zero
    control.p = 0;
    LOG(WARNING) << "Desired roll is 90 degrees. Setting pitch to 0";
  }

  if (control.t >= config_.max_thrust())
    control.t = config_.max_thrust();

  control.y = goal.yaw;
  return true;
}

ControllerStatus
RPYTBasedVelocityController::isConvergedImplementation(VelocityYaw sensor_data,
                                                       VelocityYaw goal) {
  ControllerStatus status = ControllerStatus::Active;
  VelocityYaw velocity_diff = goal - sensor_data;
  status << "Error velocity, yaw: " << velocity_diff.x << velocity_diff.y
         << velocity_diff.z << velocity_diff.yaw;
  DATA_LOG("rpyt_based_velocity_controller")
      << velocity_diff.x << velocity_diff.y << velocity_diff.z
      << velocity_diff.yaw << sensor_data.x << sensor_data.y << sensor_data.z
      << sensor_data.yaw << goal.x << goal.y << goal.z << goal.yaw
      << DataStream::endl;
  const VelocityControllerConfig &velocity_controller_config =
      config_.velocity_controller_config();
  const config::Velocity tolerance_vel =
      velocity_controller_config.goal_velocity_tolerance();
  const double tolerance_yaw = velocity_controller_config.goal_yaw_tolerance();
  // Compare
  if (std::abs(velocity_diff.x) < tolerance_vel.vx() &&
      std::abs(velocity_diff.y) < tolerance_vel.vy() &&
      std::abs(velocity_diff.z) < tolerance_vel.vz() &&
      std::abs(velocity_diff.yaw) < tolerance_yaw) {
    status.setStatus(ControllerStatus::Completed);
  }
  return status;
}
