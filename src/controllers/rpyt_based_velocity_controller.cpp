#include "aerial_autonomy/log/log.h"
#include <Eigen/Dense>
#include <aerial_autonomy/common/math.h>
#include <aerial_autonomy/controllers/rpyt_based_velocity_controller.h>
#include <glog/logging.h>

bool RPYTBasedVelocityController::runImplementation(
    std::tuple<VelocityYawRate, double> sensor_data, VelocityYawRate goal,
    RollPitchYawRateThrust &control) {
  double yaw = std::get<1>(sensor_data);
  VelocityYawRate velocity_yawrate_diff = goal - std::get<0>(sensor_data);
  cumulative_error =
      cumulative_error + velocity_yawrate_diff * controller_timer_duration_;

  RPYTBasedVelocityControllerConfig config = config_;
  // Acceleration in world frame
  double acc_x =
      config.kp() * velocity_yawrate_diff.x + config.ki() * cumulative_error.x;
  double acc_y =
      config.kp() * velocity_yawrate_diff.y + config.ki() * cumulative_error.y;
  double acc_z = config.kp() * velocity_yawrate_diff.z +
                 config.ki() * cumulative_error.z + 9.81;

  // Acceleration in gravity aligned yaw-compensated frame
  Eigen::Vector3d rot_acc;
  rot_acc[0] = acc_x * cos(yaw) + acc_y * sin(yaw);
  rot_acc[1] = -acc_x * sin(yaw) + acc_y * cos(yaw);
  rot_acc[2] = acc_z;

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
  DATA_LOG("rpyt_based_velocity_controller")
      << velocity_yawrate_diff.x << velocity_yawrate_diff.y
      << velocity_yawrate_diff.z << velocity_yawrate_diff.yaw_rate
      << velocity_yawrate.x << velocity_yawrate.y << velocity_yawrate.z
      << velocity_yawrate.yaw_rate << goal.x << goal.y << goal.z
      << goal.yaw_rate << DataStream::endl;
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
