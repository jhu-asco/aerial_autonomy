#include "aerial_autonomy/types/quad_particle_reference_trajectory.h"
#include "aerial_autonomy/common/conversions.h"
#include <glog/logging.h>

constexpr double QuadParticleTrajectory::gravity_magnitude_;

std::pair<Eigen::VectorXd, Eigen::VectorXd>
QuadParticleTrajectory::atTime(double t) const {
  // State: position, rpy, velocity, rpydot, rpyd
  // Controls: thrust, rpyd_dot
  Eigen::VectorXd x(21);
  Eigen::VectorXd u(6);
  double exp_kp_x = exp(-config_.kp_x() * t);
  double x_diff = (current_state_.x - goal_state_.x);
  double exp_x_diff = exp_kp_x * x_diff;
  x[0] = exp_x_diff + goal_state_.x;
  x[6] = -config_.kp_x() * exp_x_diff;
  // y
  double exp_kp_y = exp(-config_.kp_y() * t);
  double y_diff = (current_state_.y - goal_state_.y);
  double exp_y_diff = exp_kp_y * y_diff;
  x[1] = exp_y_diff + goal_state_.y;
  x[7] = -config_.kp_y() * exp_y_diff;
  // z
  double exp_kp_z = exp(-config_.kp_z() * t);
  double z_diff = (current_state_.z - goal_state_.z);
  double exp_z_diff = exp_kp_z * z_diff;
  x[2] = exp_z_diff + goal_state_.z;
  x[8] = -config_.kp_z() * exp_z_diff;
  // yaw
  double exp_kp_yaw = exp(-config_.kp_yaw() * t);
  double yaw_diff = (current_state_.yaw - goal_state_.yaw);
  double exp_yaw_diff = exp_kp_yaw * yaw_diff;
  x[5] = exp_yaw_diff + goal_state_.yaw;
  x[11] = -config_.kp_yaw() * exp_yaw_diff;
  // Acc
  Eigen::Vector3d acc(-config_.kp_x() * x[6], -config_.kp_y() * x[7],
                      -config_.kp_z() * x[8]);
  auto roll_pitch = conversions::accelerationToRollPitch(x[5], acc);
  x[3] = roll_pitch.first;
  x[4] = roll_pitch.second;
  // Rpdot
  double dt = 1e-6;
  double t_dt = t + dt;
  // x
  double exp_kp_x_dt = exp(-config_.kp_x() * (t_dt));
  double acc_x_dt = (config_.kp_x() * config_.kp_x()) * exp_kp_x_dt * x_diff;
  // y
  double exp_kp_y_dt = exp(-config_.kp_y() * (t_dt));
  double acc_y_dt = (config_.kp_y() * config_.kp_y()) * exp_kp_y_dt * y_diff;
  // z
  double exp_kp_z_dt = exp(-config_.kp_z() * (t_dt));
  double acc_z_dt = (config_.kp_z() * config_.kp_z()) * exp_kp_z_dt * z_diff;
  // yaw
  double exp_kp_yaw_dt = exp(-config_.kp_yaw() * (t_dt));
  double yaw_dt = exp_kp_yaw_dt * yaw_diff + goal_state_.yaw;
  Eigen::Vector3d acc_dt(acc_x_dt, acc_y_dt, acc_z_dt);
  auto roll_pitch_dt = conversions::accelerationToRollPitch(yaw_dt, acc_dt);
  x[9] = (roll_pitch_dt.first - x[3]) / dt;
  x[10] = (roll_pitch_dt.second - x[4]) / dt;
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
