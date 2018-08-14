#include "aerial_autonomy/controllers/rpyt_based_position_controller.h"
bool RPYTBasedPositionController::runImplementation(
    std::tuple<VelocityYawRate, PositionYaw> sensor_data, PositionYaw goal,
    RollPitchYawRateThrust &control) {
  auto velocity = std::get<0>(sensor_data);
  auto position = std::get<1>(sensor_data);

  VelocityYawRate velocity_command;
  position_controller_.setGoal(goal, true);
  bool control_success = position_controller_.run(position, velocity_command);
  if (control_success) {
    rpyt_velocity_controller_.setGoal(velocity_command);
    control_success &= rpyt_velocity_controller_.run(
        std::make_tuple(velocity, position.yaw), control);
  }
  return control_success;
}
ControllerStatus RPYTBasedPositionController::isConvergedImplementation(
    std::tuple<VelocityYawRate, PositionYaw> sensor_data, PositionYaw goal) {
  auto velocity = std::get<0>(sensor_data);
  auto position = std::get<1>(sensor_data);
  ControllerStatus controller_status(ControllerStatus::Completed);
  controller_status += rpyt_velocity_controller_.isConverged(
      std::make_tuple(velocity, position.yaw));
  position_controller_.setGoal(goal, true);
  controller_status += position_controller_.isConverged(position);
  return controller_status;
}
