#include <aerial_autonomy/common/math.h>
#include <aerial_autonomy/controllers/velocity_based_position_controller.h>
#include <glog/logging.h>

VelocityYaw VelocityBasedPositionController::runImplementation(
    PositionYaw sensor_data, PositionYaw goal, ControllerStatus &) {
  PositionYaw position_diff = goal - sensor_data;
  // \todo Matt account for wrapping
  auto yaw_cmd = math::angleWrap(
      sensor_data.yaw + math::clamp(config_.yaw_gain() * position_diff.yaw,
                                    -config_.max_yaw_rate(),
                                    config_.max_yaw_rate()));
  double position_norm = position_diff.position().norm();
  double velocity =
      std::min(config_.max_velocity(), config_.position_gain() * position_norm);
  return VelocityYaw(velocity * position_diff.x / position_norm,
                     velocity * position_diff.y / position_norm,
                     velocity * position_diff.z / position_norm, yaw_cmd);
}

bool VelocityBasedPositionController::isConvergedImplementation(
    PositionYaw sensor_data, PositionYaw goal) {
  PositionYaw position_diff = goal - sensor_data;
  const PositionControllerConfig &position_controller_config =
      config_.position_controller_config();
  const double &tolerance_pos =
      position_controller_config.goal_position_tolerance();
  const double &tolerance_yaw = position_controller_config.goal_yaw_tolerance();
  // Compare
  if (std::abs(position_diff.x) < tolerance_pos &&
      std::abs(position_diff.y) < tolerance_pos &&
      std::abs(position_diff.z) < tolerance_pos &&
      std::abs(position_diff.yaw) < tolerance_yaw) {
    VLOG(1) << "Reached goal";
    return true;
  }
  return false;
}