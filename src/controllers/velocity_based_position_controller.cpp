#include <aerial_autonomy/common/math.h>
#include <aerial_autonomy/controllers/velocity_based_position_controller.h>
#include <glog/logging.h>

void VelocityBasedPositionController::resetIntegrator() {
  cumulative_error = PositionYaw(0, 0, 0, 0);
}

double VelocityBasedPositionController::backCalculate(
    double &integrator, const double &p_command, const double &saturation) {
  double command = p_command + integrator;
  if (command > saturation) {
    command = saturation;
    integrator = saturation - p_command;
  } else if (command < -saturation) {
    command = -saturation;
    integrator = -saturation - p_command;
  }
  return command;
}

bool VelocityBasedPositionController::runImplementation(
    PositionYaw sensor_data, PositionYaw goal, VelocityYawRate &control) {
  PositionYaw position_diff = goal - sensor_data;
  PositionYaw p_position_diff(position_diff.position() *
                                  config_.position_gain(),
                              position_diff.yaw * config_.yaw_gain());
  PositionYaw i_position_diff(position_diff.position() *
                                  config_.position_i_gain(),
                              position_diff.yaw * config_.yaw_i_gain());
  cumulative_error = cumulative_error + i_position_diff * dt;

  control.x = backCalculate(cumulative_error.x, p_position_diff.x,
                            config_.max_velocity());
  control.y = backCalculate(cumulative_error.y, p_position_diff.y,
                            config_.max_velocity());
  control.z = backCalculate(cumulative_error.z, p_position_diff.z,
                            config_.max_velocity());
  control.yaw_rate = backCalculate(cumulative_error.yaw, p_position_diff.yaw,
                                   config_.max_yaw_rate());

  DATA_LOG("velocity_based_position_controller")
      << position_diff.x << position_diff.y << position_diff.z
      << position_diff.yaw << cumulative_error.x << cumulative_error.y
      << cumulative_error.z << cumulative_error.yaw << control.x << control.y
      << control.z << control.yaw_rate << DataStream::endl;
  return true;
}

ControllerStatus VelocityBasedPositionController::isConvergedImplementation(
    PositionYaw sensor_data, PositionYaw goal) {
  PositionYaw position_diff = goal - sensor_data;
  ControllerStatus status(ControllerStatus::Active);
  status << "Error Position, Yaw: " << position_diff.x << position_diff.y
         << position_diff.z << position_diff.yaw;
  const PositionControllerConfig &position_controller_config =
      config_.position_controller_config();
  const config::Position &tolerance_pos =
      position_controller_config.goal_position_tolerance();
  const double &tolerance_yaw = position_controller_config.goal_yaw_tolerance();
  // Compare
  if (std::abs(position_diff.x) < tolerance_pos.x() &&
      std::abs(position_diff.y) < tolerance_pos.y() &&
      std::abs(position_diff.z) < tolerance_pos.z() &&
      std::abs(position_diff.yaw) < tolerance_yaw) {
    VLOG_EVERY_N(1, 50) << "Reached goal";
    status.setStatus(ControllerStatus::Completed, "Reached goal");
  }
  return status;
}
