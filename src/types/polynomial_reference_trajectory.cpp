#include "aerial_autonomy/types/polynomial_reference_trajectory.h"
#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/common/math.h"
#include <glog/logging.h>

constexpr double PolynomialReferenceTrajectory::gravity_magnitude_;

PolynomialReferenceTrajectory::PolynomialReferenceTrajectory(
    PositionYaw goal_state, PositionYaw start_state,
    PolynomialReferenceConfig config)
    : degree_(9), dimensions_(4), goal_state_(goal_state),
      start_state_(dimensions_), config_(config) {
  PositionYaw error = goal_state - start_state;
  start_state_ << start_state.x, start_state.y, start_state.z, start_state.yaw;
  Eigen::MatrixXd constraints(degree_ + 1, dimensions_);
  Eigen::MatrixXd basis(degree_ + 1, degree_ + 1);
  constraints.setZero();
  constraints(0, 0) = error.x;
  constraints(0, 1) = error.y;
  constraints(0, 2) = error.z;
  constraints(0, 3) = error.yaw;
  CHECK_GT(config.min_tf(), 1e-2) << "Final time should be greater than 1e-2";
  CHECK_LT(config.max_velocity(), 2.0)
      << "Final time should be greater than 1e-2";
  tf_ = std::max(Eigen::Vector3d(error.x, error.y, error.z).norm() /
                     config.max_velocity(),
                 config.min_tf());
  VLOG_EVERY_N(2, 20) << "Tf: " << tf_;
  basis.topRows(dimensions_ + 1) = findBasisMatrix(tf_, degree_, dimensions_);
  basis.bottomRows(dimensions_ + 1) =
      findBasisMatrix(0.0, degree_, dimensions_);
  coefficients_ = basis.colPivHouseholderQr().solve(constraints);
}

Eigen::MatrixXd
PolynomialReferenceTrajectory::findBasisMatrix(double t, int degree,
                                               int dimensions) const {
  Eigen::MatrixXd basis(dimensions + 1, degree + 1);
  Eigen::VectorXd coeff(degree + 1);
  coeff.setOnes();
  Eigen::VectorXd t_exp(degree + 1);
  t_exp(0) = 1;
  for (int i = 1; i < degree + 1; ++i) {
    t_exp(i) = t_exp(i - 1) * t;
  }
  for (int row = 0; row < dimensions + 1; ++row) {
    for (int col = 0; col < degree + 1; ++col) {
      int col_row_diff = col - row;
      if (row >= 1 && col_row_diff >= 0) {
        coeff(col) = coeff(col) * (col_row_diff + 1);
      }
      if (col_row_diff >= 0) {
        basis(row, col) = t_exp(col_row_diff) * coeff(col);
      } else {
        basis(row, col) = 0;
      }
    }
  }
  return basis;
}

Eigen::Vector3d PolynomialReferenceTrajectory::getNoise(double t, double a,
                                                        double nu) const {
  double omega = nu * M_PI * 2.0;
  double s_omega_t = sin(omega * t), c_omega_t = cos(omega * t);
  double s_omega_t_squared = std::pow(s_omega_t, 2);
  double s_omega_t_cubed = s_omega_t_squared * s_omega_t;
  double s_omega_t_quadrupled = s_omega_t_cubed * s_omega_t;
  double omega_squared = std::pow(omega, 2);
  Eigen::Vector3d noise;
  noise[0] = a * s_omega_t_quadrupled;
  noise[1] = 4 * omega * a * s_omega_t_cubed * c_omega_t;
  noise[2] = -16 * omega_squared * noise[0] +
             12 * omega_squared * a * s_omega_t_squared;
  return noise;
}

std::pair<Eigen::VectorXd, Eigen::VectorXd>
PolynomialReferenceTrajectory::atTime(double t) const {
  Eigen::VectorXd state(15);
  Eigen::VectorXd control(4);
  double t_clamped = math::clamp(t, 0, tf_);
  Eigen::MatrixXd basis = findBasisMatrix(t_clamped, degree_, dimensions_);
  Eigen::MatrixXd out = basis * coefficients_;
  Eigen::VectorXd position_yaw = start_state_ + out.row(0).transpose();
  Eigen::VectorXd velocity_yawrate = out.row(1);
  Eigen::VectorXd acceleration_yaw = out.row(2);
  if (t > tf_ && config_.add_noise()) {
    double dt = t - tf_;
    Eigen::Vector3d forward_noise =
        getNoise(dt, config_.forward_noise_amplitude(),
                 config_.forward_noise_frequency());
    Eigen::Vector3d z_noise =
        getNoise(dt, config_.z_noise_amplitude(), config_.z_noise_frequency());
    double c_yaw = cos(goal_state_.yaw), s_yaw = sin(goal_state_.yaw);
    position_yaw[0] += forward_noise[0] * c_yaw;
    position_yaw[1] += forward_noise[0] * s_yaw;
    position_yaw[2] += z_noise[0];
    // vel
    velocity_yawrate[0] += forward_noise[1] * c_yaw;
    velocity_yawrate[1] += forward_noise[1] * s_yaw;
    velocity_yawrate[2] += z_noise[1];
    // acc
    acceleration_yaw[0] += forward_noise[2] * c_yaw;
    acceleration_yaw[1] += forward_noise[2] * s_yaw;
    acceleration_yaw[2] += z_noise[2];
  }
  Eigen::Vector3d acceleration = acceleration_yaw.segment(0, 3);
  // wrap yaw
  position_yaw(3) = math::angleWrap(position_yaw(3));
  // Compensate gravity
  acceleration[2] = acceleration[2] + gravity_magnitude_;

  // Get rp from acceleration
  auto roll_pitch =
      conversions::accelerationToRollPitch(position_yaw(3), acceleration);
  // Fill state
  state.segment(0, 3) = position_yaw.segment<3>(0);     // pos
  state.segment(6, 3) = velocity_yawrate.segment<3>(0); // vel
  ///\todo fill rp_rate correctly
  state.segment(9, 2).setZero();   // rp_rate
  state(11) = velocity_yawrate(3); // yaw_rate
  // rpy, rpy_cmd
  state(12) = state(3) = roll_pitch.first;
  state(13) = state(4) = roll_pitch.second;
  state(14) = state(5) = position_yaw(3);
  // Fill control
  control(0) = acceleration.norm() / gravity_magnitude_;
  control(1) = control(2) = 0; // rp_rate \\\todo fill correctly
  control(3) = velocity_yawrate(3);
  return std::make_pair(state, control);
}

Eigen::VectorXd PolynomialReferenceTrajectory::goal(double) {
  Eigen::VectorXd goal_state(15);
  goal_state.setZero();
  goal_state[0] = goal_state_.x;
  goal_state[1] = goal_state_.y;
  goal_state[2] = goal_state_.z;
  goal_state[5] = goal_state_.yaw;
  goal_state[14] = goal_state_.yaw;
  return goal_state;
}
