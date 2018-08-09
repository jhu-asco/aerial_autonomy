#include "aerial_autonomy/types/quad_particle_reference_trajectory.h"
#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/common/math.h"
#include <glog/logging.h>

constexpr double QuadParticleTrajectory::gravity_magnitude_,
    QuadParticleTrajectory::tol;

QuadParticleTrajectory::QuadParticleTrajectory(PositionYaw goal_state,
                                               PositionYaw current_state,
                                               ParticleReferenceConfig config)
    : ParticleTrajectory<Eigen::VectorXd, Eigen::VectorXd>(
          goal_state, current_state, config) {
  error = goal_state - current_state;
  time_intersection[0] =
      findTimeIntersection(config.kp_x(), error.x, config.max_velocity());
  time_intersection[1] =
      findTimeIntersection(config.kp_y(), error.y, config.max_velocity());
  time_intersection[2] =
      findTimeIntersection(config.kp_z(), error.z, config.max_velocity());
  time_intersection[3] =
      findTimeIntersection(config.kp_yaw(), error.yaw, config.max_yaw_rate());
  linear_slopes = Eigen::Vector4d::Zero();
  if (time_intersection[0] > tol) {
    linear_slopes[0] = std::copysign(config.max_velocity(), error.x);
  }
  if (time_intersection[1] > tol) {
    linear_slopes[1] = std::copysign(config.max_velocity(), error.y);
  }
  if (time_intersection[2] > tol) {
    linear_slopes[2] = std::copysign(config.max_velocity(), error.z);
  }
  if (time_intersection[3] > tol) {
    linear_slopes[3] = std::copysign(config.max_yaw_rate(), error.yaw);
  }
}

double QuadParticleTrajectory::findTimeIntersection(double gain, double x_diff,
                                                    double max_velocity) {
  double gain_x_diff = gain * std::abs(x_diff);
  if (gain_x_diff > max_velocity) {
    double gain_t = log(gain_x_diff / max_velocity);
    return gain_t / gain;
  } else {
    return 0;
  }
}

std::pair<Eigen::VectorXd, Eigen::VectorXd>
QuadParticleTrajectory::atTime(double t) const {
  // State: position, rpy, velocity, rpydot, rpyd
  // Controls: thrust, rpyd_dot
  Eigen::VectorXd x(15);
  Eigen::VectorXd u(4);
  Eigen::Vector3d acc;    // Acc
  Eigen::Vector3d acc_dt; // Acc at t+dt
  double yaw_dt;          // Yaw at t+dt
  double dt = 1e-6;
  double t_dt = t + dt;
  // x
  double exp_kp_x = exp(-config_.kp_x() * t);
  double exp_kp_x_dt = exp(-config_.kp_x() * (t_dt));
  double exp_x_diff = -exp_kp_x * error.x;
  if (t < time_intersection[0]) {
    x[0] = current_state_.x + linear_slopes[0] * t;
    x[6] = linear_slopes[0];
    acc[0] = 0;
    acc_dt[0] = 0;
  } else {
    x[0] = exp_x_diff + goal_state_.x;
    x[6] = -config_.kp_x() * exp_x_diff;
    acc[0] = -config_.kp_x() * x[6];
    acc_dt[0] = -(config_.kp_x() * config_.kp_x()) * exp_kp_x_dt * error.x;
  }
  // y
  double exp_kp_y = exp(-config_.kp_y() * t);
  double exp_kp_y_dt = exp(-config_.kp_y() * (t_dt));
  double exp_y_diff = -exp_kp_y * error.y;
  if (t < time_intersection[1]) {
    x[1] = current_state_.y + linear_slopes[1] * t;
    x[7] = linear_slopes[1];
    acc[1] = 0;
    acc_dt[1] = 0;
  } else {
    x[1] = exp_y_diff + goal_state_.y;
    x[7] = -config_.kp_y() * exp_y_diff;
    acc[1] = -config_.kp_y() * x[7];
    acc_dt[1] = -(config_.kp_y() * config_.kp_y()) * exp_kp_y_dt * error.y;
  }
  // z
  double exp_kp_z = exp(-config_.kp_z() * t);
  double exp_kp_z_dt = exp(-config_.kp_z() * (t_dt));
  double exp_z_diff = -exp_kp_z * error.z;
  if (t < time_intersection[2]) {
    x[2] = current_state_.z + linear_slopes[2] * t;
    x[8] = linear_slopes[2];
    acc[2] = gravity_magnitude_;
    acc_dt[2] = gravity_magnitude_;
  } else {
    x[2] = exp_z_diff + goal_state_.z;
    x[8] = -config_.kp_z() * exp_z_diff;
    acc[2] = -config_.kp_z() * x[8] + gravity_magnitude_;
    acc_dt[2] = -(config_.kp_z() * config_.kp_z()) * exp_kp_z_dt * error.z +
                gravity_magnitude_;
  }
  // yaw
  double exp_kp_yaw = exp(-config_.kp_yaw() * t);
  double exp_kp_yaw_dt = exp(-config_.kp_yaw() * (t_dt));
  double exp_yaw_diff = -exp_kp_yaw * error.yaw;
  if (t < time_intersection[3]) {
    x[5] = current_state_.yaw + linear_slopes[3] * t;
    x[11] = linear_slopes[3];
    yaw_dt = current_state_.yaw + linear_slopes[3] * (t_dt);
  } else {
    x[5] = exp_yaw_diff + goal_state_.yaw;
    x[11] = -config_.kp_yaw() * exp_yaw_diff;
    yaw_dt = -exp_kp_yaw_dt * error.yaw + goal_state_.yaw;
  }
  auto roll_pitch = conversions::accelerationToRollPitch(x[5], acc);
  x[3] = roll_pitch.first;
  x[4] = roll_pitch.second;
  // Rpdot
  auto roll_pitch_dt = conversions::accelerationToRollPitch(yaw_dt, acc_dt);
  x[9] = (roll_pitch_dt.first - x[3]) / dt;
  x[10] = (roll_pitch_dt.second - x[4]) / dt;
  // Wrap yaw
  x[5] = math::angleWrap(x[5]);
  // Commanded angles
  x[12] = x[3];
  x[13] = x[4];
  x[14] = x[5];

  // Controls
  u[0] = acc.norm() / gravity_magnitude_;
  // rpyd_dot = rpy_dot
  u[1] = x[9];
  u[2] = x[10];
  u[3] = x[11];
  return std::make_pair(x, u);
}
