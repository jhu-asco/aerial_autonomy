#include "aerial_autonomy/log/log.h"
#include <Eigen/Dense>
#include <aerial_autonomy/common/math.h>
#include <aerial_autonomy/controllers/rpyt_based_velocity_controller.h>
#include <glog/logging.h>

bool RPYTBasedVelocityController::runImplementation(
    std::tuple<VelocityYawRate, double> sensor_data, VelocityYawRate goal,
    RollPitchYawRateThrust &control) {
  double yaw = std::get<1>(sensor_data);
  VelocityYawRate velocity_yawrate = std::get<0>(sensor_data);
  VelocityYawRate velocity_yawrate_diff = goal - velocity_yawrate;
  cumulative_error = cumulative_error +
                     velocity_yawrate_diff * controller_timer_duration_.count();

  RPYTBasedVelocityControllerConfig config = config_;
  // Acceleration in world frame
  Eigen::Vector3d world_acc;
  world_acc(0) = config.kp_xy() * velocity_yawrate_diff.x +
                 config.ki_xy() * cumulative_error.x;
  world_acc(1) = config.kp_xy() * velocity_yawrate_diff.y +
                 config.ki_xy() * cumulative_error.y;
  world_acc(2) = config.kp_z() * velocity_yawrate_diff.z +
                 config.ki_z() * cumulative_error.z;
  // Limit acceleration magnitude
  double acc_norm = world_acc.norm();
  if (acc_norm > config.max_acc_norm()) {
    world_acc *= (config.max_acc_norm() / acc_norm);
  }
  // Compensate for gravity after limiting the residual
  world_acc(2) += 9.81;

  // Acceleration in gravity aligned yaw-compensated frame
  Eigen::Vector3d rot_acc;
  rot_acc[0] = world_acc(0) * cos(yaw) + world_acc(1) * sin(yaw);
  rot_acc[1] = -world_acc(0) * sin(yaw) + world_acc(1) * cos(yaw);
  rot_acc[2] = world_acc(2);

  // thrust is magnitude of acceleration scaled by kt
  control.t = rot_acc.norm() / config.kt();

  // normalize acceleration in gravity aligned yaw-compensated frame
  if (control.t > 1e-8)
    rot_acc = (1 / (config.kt() * control.t)) * rot_acc;
  else {
    LOG(WARNING) << "Thrust close to zero !!";
    rot_acc = Eigen::Vector3d(0, 0, 0);
  }

  // yaw-compensated y-acceleration is sine of roll
  control.r = -asin(rot_acc[1]);

  // check if roll = 90 and compute pitch accordingly
  if ((abs(rot_acc[1]) - 1.0) < config.tolerance_rp())
    control.p = atan2(rot_acc[0], rot_acc[2]);
  else {
    // if roll is 90, pitch is undefined and is set to zero
    control.p = 0;
    LOG(WARNING) << "Desired roll is 90 degrees. Setting pitch to 0";
  }

  control.t = math::clamp(control.t, config.min_thrust(), config.max_thrust());
  control.r = math::clamp(control.r, -config.max_rp(), config.max_rp());
  control.p = math::clamp(control.p, -config.max_rp(), config.max_rp());

  control.y = goal.yaw_rate;
  DATA_LOG("rpyt_based_velocity_controller")
      << velocity_yawrate_diff.x << velocity_yawrate_diff.y
      << velocity_yawrate_diff.z << velocity_yawrate_diff.yaw_rate
      << velocity_yawrate.x << velocity_yawrate.y << velocity_yawrate.z
      << velocity_yawrate.yaw_rate << goal.x << goal.y << goal.z
      << goal.yaw_rate << world_acc[0] << world_acc[1] << world_acc[2]
      << DataStream::endl;
  ///\todo  Add cumulative error to yaw rate (integrator)
  return true;
}

ControllerStatus RPYTBasedVelocityController::isConvergedImplementation(
    std::tuple<VelocityYawRate, double> sensor_data, VelocityYawRate goal) {
  ControllerStatus status = ControllerStatus::Active;
  VelocityYawRate velocity_yawrate = std::get<0>(sensor_data);
  VelocityYawRate velocity_yawrate_diff = goal - velocity_yawrate;
  status << "Error velocity, yaw rate: " << velocity_yawrate_diff.x
         << velocity_yawrate_diff.y << velocity_yawrate_diff.z
         << velocity_yawrate_diff.yaw_rate;
  RPYTBasedVelocityControllerConfig config = config_;
  const VelocityControllerConfig &velocity_controller_config =
      config.velocity_controller_config();
  const config::Velocity tolerance_vel =
      velocity_controller_config.goal_velocity_tolerance();
  const double tolerance_yaw_rate =
      velocity_controller_config.goal_yaw_rate_tolerance();
  // Compare
  if (std::abs(velocity_yawrate_diff.x) < tolerance_vel.vx() &&
      std::abs(velocity_yawrate_diff.y) < tolerance_vel.vy() &&
      std::abs(velocity_yawrate_diff.z) < tolerance_vel.vz() &&
      std::abs(velocity_yawrate_diff.yaw_rate) < tolerance_yaw_rate) {
    status.setStatus(ControllerStatus::Completed);
  }
  return status;
}
