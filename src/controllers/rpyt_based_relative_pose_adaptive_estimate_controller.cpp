#include "aerial_autonomy/controllers/rpyt_based_relative_pose_adaptive_estimate_controller.h"
#include "aerial_autonomy/log/log.h"

#include <glog/logging.h>

bool RPYTBasedRelativePoseAdaptiveEstimateController::runImplementation(
    std::tuple<double, double, ParticleStateYaw> sensor_data,
    ReferenceTrajectoryPtr<ParticleStateYaw, Snap> goal,
    RollPitchYawThrustAdaptive &control) {
  // Get the current time
  double curr_time = std::get<1>(sensor_data);
  // Get the current mhat
  double mhat = std::get<0>(sensor_data);
  // Get the goal state
  ParticleStateYaw desired_state;
  try {
    auto temp_pair = goal->atTime(curr_time);
    desired_state = temp_pair.first;
  } catch (std::logic_error e) {
    LOG(WARNING) << e.what() << std::endl;
    return false;
  }
  // Get the current state
  ParticleStateYaw state = std::get<2>(sensor_data);
  double curr_yaw = state.yaw;
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
  // The details behind this formula can be found in this document:
  // https://drive.google.com/open?id=1GYvwzfI0OKF62tpSu5e_kgDBWRzJuQ96
  control.dm =
      -km_ * ((acc_d - ag_).transpose()) * (delta_v + config_.eps() * delta_p);
  control.dm = math::clamp(control.dm, -config_.max_dm(), config_.max_dm());
  // Find body acceleration
  Eigen::Vector3d world_acc = -K_ * delta_x + mhat * acc_d;
  // Clamp each component of world_acc to plus-minus max_acc
  world_acc[0] =
      math::clamp(world_acc(0), -config_.max_acc(), config_.max_acc());
  world_acc[1] =
      math::clamp(world_acc(1), -config_.max_acc(), config_.max_acc());
  world_acc[2] =
      math::clamp(world_acc(2), -config_.max_acc(), config_.max_acc());
  // Check each velocity for exceeding the bound, and if so, zero out the
  // acceleration if it would get worse.
  for (int i = 0; i < 3; i++) {
    if ((v(i) > config_.max_vel() && world_acc(i) > 0) ||
        (v(i) < -config_.max_vel() && world_acc(i) < 0)) {
      LOG(WARNING) << "Velocity Bound Reached: " << i;
      world_acc[i] = 0;
    }
  }
  // Add in the gravity term
  world_acc = -mhat * ag_ + world_acc;
  // Rotate by yaw
  Eigen::Vector3d rot_acc;
  rot_acc[0] = world_acc(0) * cos(curr_yaw) + world_acc(1) * sin(curr_yaw);
  rot_acc[1] = -world_acc(0) * sin(curr_yaw) + world_acc(1) * cos(curr_yaw);
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
  control.y =
      -config_.k_yaw() * (math::angleWrap(curr_yaw - desired_state.yaw));
  control.y =
      math::clamp(control.y, -config_.yaw_rate_max(), config_.yaw_rate_max());
  // Estimate the lyapunov function for logging
  double lyap_V = (0.5 * (delta_x.transpose() * P_ * delta_x)).norm();
  lyap_V += 0.5 * (mhat - 6) * (mhat - 6) / (km_);
  // Log the data
  DATA_LOG("adaptive_controller")
      << mhat << lyap_V << delta_x(0) << delta_x(1) << delta_x(2) << delta_x(3)
      << delta_x(4) << delta_x(5) << (curr_yaw - desired_state.yaw)
      << desired_state.p.x << desired_state.p.y << desired_state.p.z
      << state.p.x << state.p.y << state.p.z << world_acc(0) << world_acc(1)
      << world_acc(2) << control.r << control.p << curr_time
      << DataStream::endl;
  return true;
}

ControllerStatus
RPYTBasedRelativePoseAdaptiveEstimateController::isConvergedImplementation(
    std::tuple<double, double, ParticleStateYaw> sensor_data,
    ReferenceTrajectoryPtr<ParticleStateYaw, Snap> goal) {
  // Get the current time
  double curr_time = std::get<1>(sensor_data);
  // Get the current state
  ControllerStatus status = ControllerStatus::Active;
  ParticleStateYaw state = std::get<2>(sensor_data);
  double curr_yaw = state.yaw;
  // Get the goal state
  ParticleStateYaw desired_state;
  try {
    desired_state = goal->goal(curr_time);
  } catch (std::logic_error e) {
    LOG(WARNING) << e.what() << std::endl;
    return ControllerStatus::Critical;
  }
  double delta_yaw = math::angleWrap(curr_yaw - desired_state.yaw);
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
         << std::get<0>(sensor_data) << delta_p(0) << delta_p(1) << delta_p(2);
  // Controller converged if position, velocity, and yaw are within tolerances
  // and if the reference trajectory acc is below the tolerance, so we don't
  // quit early
  if (delta_p.norm() < config_.tolerance_pos() &&
      delta_v.norm() < config_.tolerance_vel() &&
      delta_yaw < config_.tolerance_yaw() &&
      acc_d.norm() < config_.tolerance_acc()) {
    status.setStatus(ControllerStatus::Completed);
  }
  return status;
}
