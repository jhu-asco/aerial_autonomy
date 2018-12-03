#include "aerial_autonomy/controllers/rpyt_based_relative_pose_adaptive_estimate_controller.h"
#include "aerial_autonomy/log/log.h"

#include <glog/logging.h>

bool RPYTBasedRelativePoseAdaptiveEstimateController::runImplementation(
    std::pair<double, ParticleState> sensor_data,
    std::pair<ReferenceTrajectoryPtr<ParticleState, Snap>, double> goal,
    RollPitchYawThrustAdaptive &control) {
  // Get the current mhat
  double mhat = std::get<0>(sensor_data);
  // Get the goal state
  ParticleState desired_state;
  auto current_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> dt_duration =
      std::chrono::duration_cast<std::chrono::duration<double>>(current_time -
                                                                t0_);
  double curr_time = dt_duration.count();
  try {
    auto temp_pair = std::get<0>(goal)->atTime(curr_time);
    desired_state = temp_pair.first;
  } catch (std::logic_error e) {
    LOG(WARNING) << e.what() << std::endl;
    return false;
  }
  double yaw = std::get<1>(goal);
  // Get the current state
  ParticleState state = std::get<1>(sensor_data);
  // Unpack into Eigen structures
  Eigen::Vector3d p(state.p.x, state.p.y, state.p.z);
  Eigen::Vector3d p_d(desired_state.p.x, desired_state.p.y, desired_state.p.z);
  Eigen::Vector3d v(state.v.x, state.v.y, state.v.z);
  Eigen::Vector3d v_d(desired_state.v.x, desired_state.v.y, desired_state.v.z);
  Eigen::Vector3d acc_d(desired_state.a.x, desired_state.a.y,
                        desired_state.a.z);

  Vector6d x, x_d, delta_x;
  x.head<3>() = p;
  x.tail<3>() = v;
  x_d.head<3>() = p_d;
  x_d.tail<3>() = v_d;
  delta_x = x - x_d;
  Eigen::Vector3d delta_p = p - p_d;
  Eigen::Vector3d delta_v = v - v_d;

  // Update mhat
  control.dm =
      -km * ((acc_d - ag_).transpose()) * (delta_v + config_.eps() * delta_p);
  control.dm = math::clamp(control.dm, -config_.max_dm(), config_.max_dm());
  /*double delta_z = delta_p(2);
  Eigen::Vector3d kp(config_.kp_xy(), config_.kp_xy(), config_.kp_z());
  double delta_m_est = mhat - 6;
  if (delta_z > -((delta_p.transpose() * kp.asDiagonal() *
  delta_p).norm())/(km*ag_(2)*delta_m_est)){
    control.dm = 0;
  }*/
  // Find body acceleration
  auto world_acc = (mhat * acc_d - mhat * ag_ - K_ * delta_x);
  Eigen::Vector3d rot_acc;
  rot_acc[0] = world_acc(0) * cos(yaw) + world_acc(1) * sin(yaw);
  rot_acc[1] = -world_acc(0) * sin(yaw) + world_acc(1) * cos(yaw);
  rot_acc[2] = world_acc(2);
  // Find thrust
  control.t = rot_acc.norm();
  // normalize acceleration in gravity aligned yaw-compensated frame
  if (control.t > 1e-8)
    rot_acc = (1 / rot_acc.norm()) * rot_acc;
  else {
    LOG(WARNING) << "Thrust close to zero !!" << control.t;
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
  // Clamp values according to config
  control.t =
      math::clamp(control.t, config_.min_thrust(), config_.max_thrust());
  control.r = math::clamp(control.r, -config_.max_rp(), config_.max_rp());
  control.p = math::clamp(control.p, -config_.max_rp(), config_.max_rp());
  control.y = yaw;
  double lyap_V = (0.5 * (delta_x.transpose() * P_ * delta_x)).norm();
  lyap_V += 0.5 * (mhat - 6) * (mhat - 6) / (km);

  // LOG(WARNING) << "M_Hat: " << mhat << " V: " << lyap_V << std::endl;
  DATA_LOG("adaptive_controller")
      << mhat << " " << lyap_V << " " << delta_x(0) << " " << delta_x(1) << " "
      << delta_x(2) << " " << delta_x(3) << " " << delta_x(4) << " "
      << delta_x(5) << " " << yaw << " " << DataStream::endl;
  return true;
}

ControllerStatus
RPYTBasedRelativePoseAdaptiveEstimateController::isConvergedImplementation(
    std::pair<double, ParticleState> sensor_data,
    std::pair<ReferenceTrajectoryPtr<ParticleState, Snap>, double> goal) {
  ControllerStatus status = ControllerStatus::Active;
  ParticleState state = std::get<1>(sensor_data);
  // Get the goal state
  ParticleState desired_state;
  auto current_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> dt_duration =
      std::chrono::duration_cast<std::chrono::duration<double>>(current_time -
                                                                t0_);
  double curr_time = dt_duration.count();
  try {
    auto temp_pair = std::get<0>(goal)->atTime(curr_time);
    desired_state = temp_pair.first;
  } catch (std::logic_error e) {
    LOG(WARNING) << e.what() << std::endl;
    return ControllerStatus::Critical;
  }
  // Unpack into Eigen structures
  Eigen::Vector3d p(state.p.x, state.p.y, state.p.z);
  Eigen::Vector3d p_d(desired_state.p.x, desired_state.p.y, desired_state.p.z);
  Eigen::Vector3d v(state.v.x, state.v.y, state.v.z);
  Eigen::Vector3d v_d(desired_state.v.x, desired_state.v.y, desired_state.v.z);
  Eigen::Vector3d acc_d(desired_state.a.x, desired_state.a.y,
                        desired_state.a.z);
  Eigen::Vector3d delta_p = p - p_d;
  Eigen::Vector3d delta_v = v - v_d;
  status << "Parameter Estimate and Position Error: "
         << std::get<0>(sensor_data) << "Position Error: " << delta_p(0)
         << delta_p(1) << delta_p(2);
  if (delta_p.norm() < config_.tolerance_pos() &&
      delta_v.norm() < config_.tolerance_vel() &&
      acc_d.norm() < config_.tolerance_acc()) {
    status.setStatus(ControllerStatus::Completed);
  }
  return status;
}
