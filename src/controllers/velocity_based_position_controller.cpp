#include <aerial_autonomy/controllers/velocity_based_position_controller.h>

VelocityYaw
VelocityBasedPositionController::runImplementation(PositionYaw sensor_data,
                                                   PositionYaw goal) {
  PositionYaw position_diff = goal - sensor_data;
  auto yaw_cmd = sensor_data.yaw +
                 std::min(std::max(config_.yaw_gain() * position_diff.yaw,
                                   -config_.max_yaw_rate()),
                          config_.max_yaw_rate());
  // \todo (matt) add max velocity and possibly factor in control rate for
  // yaw_cmd
  return VelocityYaw(config_.position_gain() * position_diff.x,
                     config_.position_gain() * position_diff.y,
                     config_.position_gain() * position_diff.z, yaw_cmd);
}
