#include "aerial_autonomy/types/spiral_reference_trajectory.h"
#include <glog/logging.h>

constexpr double SpiralReferenceTrajectory::epsilon;
constexpr double SpiralReferenceTrajectory::gravity_magnitude_;

SpiralReferenceTrajectory::SpiralReferenceTrajectory(
    SpiralReferenceTrajectoryConfig config, ArmSineControllerConfig arm_config,
    Eigen::Vector3d current_position, double current_yaw)
    : ReferenceTrajectory(), config_(config), arm_config_(arm_config),
      current_position_(current_position), current_yaw_(current_yaw) {
  const auto &joint_config_vec = arm_config_.joint_config();
  CHECK(joint_config_vec.size() == 2) << "There should be two joints";
  CHECK(config.frequency_z() > 0)
      << "Z frequency cannot be 0 which implies z amplitude is infinity";
}

void SpiralReferenceTrajectory::getRP(
    double &roll, double &pitch, double yaw,
    Eigen::Vector3d acceleration_vector) const {
  Eigen::Vector3d unit_vec = acceleration_vector.normalized();
  double s_yaw = sin(yaw);
  double c_yaw = cos(yaw);
  double temp1 = unit_vec[0] * s_yaw - unit_vec[1] * c_yaw;
  if (std::abs(temp1) > 1.0) {
    LOG(WARNING) << "sin(roll) > 1";
    temp1 = std::copysign(1.0, temp1);
  }
  roll = std::asin(temp1);
  double temp2 = unit_vec[0] * c_yaw + unit_vec[1] * s_yaw;
  if (std::abs(temp2) < epsilon && std::abs(unit_vec[2]) < epsilon) {
    LOG(WARNING) << "Roll is +/-90degrees which is a singularity";
    pitch = 0;
  } else {
    double cos_inv = 1.0 / cos(roll);
    pitch = std::atan2(temp2 * cos_inv, unit_vec[2] * cos_inv);
  }
}

Eigen::Vector3d SpiralReferenceTrajectory::getAcceleration(double angle,
                                                           double omega_squared,
                                                           double rx,
                                                           double ry) const {
  // Counter gravity in addition to trajectory acceleration
  return Eigen::Vector3d(-rx * omega_squared * sin(angle),
                         -ry * omega_squared * cos(angle), gravity_magnitude_);
}

std::pair<Eigen::VectorXd, Eigen::VectorXd>
SpiralReferenceTrajectory::atTime(double t) const {
  // State: position, rpy, velocity, rpydot, rpyd, ja, jv, jad
  // Controls: thrust, rpyd_dot, jad_dot;
  Eigen::VectorXd x(21);
  Eigen::VectorXd u(6);
  const double rx = config_.radiusx();
  const double ry = config_.radiusy();
  const double omega = 2 * M_PI * config_.frequency();
  const double phase = config_.phase();
  const double velocity_z = config_.velocity_z();
  const double frequency_z = config_.frequency_z();
  const double frequency_yaw = config_.frequency_yaw();
  const double amplitude_yaw = config_.amplitude_yaw();
  double amplitude_z = velocity_z / frequency_z;
  double z_rem = std::fmod(t * frequency_z, 2.0);
  double sign = z_rem < 1.0 ? 1.0 : -1.0;
  double dist_z = z_rem < 1.0 ? z_rem : (2.0 - z_rem);
  double angle = omega * t + phase;
  double omega_yaw = 2 * M_PI * frequency_yaw;
  double angle_yaw = omega_yaw * t;
  double omega_squared = omega * omega;
  double signed_velocity_z = sign * velocity_z;
  double yaw = current_yaw_ + amplitude_yaw * sin(angle_yaw);
  // Position
  x[0] = current_position_[0] + rx * sin(angle);
  x[1] = current_position_[1] + ry * cos(angle);
  x[2] = current_position_[2] + amplitude_z * dist_z;
  // Rpy
  Eigen::Vector3d acceleration_vector =
      getAcceleration(angle, omega_squared, rx, ry);
  getRP(x[3], x[4], yaw, acceleration_vector);
  x[5] = yaw;
  // Velocities
  x[6] = rx * omega * cos(angle);
  x[7] = -ry * omega * sin(angle);
  x[8] = signed_velocity_z;
  // Finite difference to get rpydot
  double dt = 1e-6;
  double angle_delta = angle + omega * dt;
  Eigen::Vector3d acceleration_vector_dt =
      getAcceleration(angle_delta, omega_squared, rx, ry);
  double yaw_dt = yaw + amplitude_yaw * sin(angle_yaw + omega_yaw * dt);
  double roll_dt, pitch_dt;
  getRP(roll_dt, pitch_dt, yaw_dt, acceleration_vector_dt);
  x[9] = (roll_dt - x[3]) / dt;
  x[10] = (pitch_dt - x[4]) / dt;
  x[11] = amplitude_yaw * omega_yaw * cos(angle_yaw);
  // Commanded angles
  x[12] = x[3];
  x[13] = x[4];
  x[14] = x[5];
  // Joint state
  const auto &joint_config_vec = arm_config_.joint_config();
  int start = 15;
  // Assuming 2 joints
  for (int i = 0; i < 2; ++i) {
    const auto &joint_config = joint_config_vec.Get(i);
    const double &amp = joint_config.amplitude();
    const double &phi = joint_config.phase();
    const double &offset = joint_config.offset();
    double omega = 2 * M_PI * joint_config.frequency();
    double angle = omega * t + phi;
    x[start + i] = offset + amp * sin(angle);
    x[start + i + 2] = amp * omega * cos(angle);
    x[start + i + 4] = x[start + i];
  }
  // Controls
  u[0] = acceleration_vector.norm() / gravity_magnitude_;
  // rpyd_dot = rpy_dot
  u[1] = x[9];
  u[2] = x[10];
  u[3] = x[11];
  // jad_dot = ja_dot
  u[4] = x[17];
  u[5] = x[18];
  return std::make_pair(x, u);
}
