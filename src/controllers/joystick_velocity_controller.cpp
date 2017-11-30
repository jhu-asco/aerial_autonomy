#include "aerial_autonomy/controllers/joystick_velocity_controller.h"
#include "aerial_autonomy/common/math.h"

bool JoystickVelocityController::runImplementation(
    std::tuple<Joystick, VelocityYawRate, double> sensor_data, EmptyGoal goal,
    RollPitchYawRateThrust &control) {

  VelocityYawRate vel_goal =
      convertJoystickToVelocityYawRate(std::get<0>(sensor_data));

  auto vel_sensor_data =
      std::make_tuple(std::get<1>(sensor_data), std::get<2>(sensor_data));

  rpyt_velocity_controller_.setGoal(vel_goal);
  return rpyt_velocity_controller_.run(vel_sensor_data, control);
}

ControllerStatus JoystickVelocityController::isConvergedImplementation(
    std::tuple<Joystick, VelocityYawRate, double> sensor_data, EmptyGoal) {
  auto vel_sensor_data =
      std::make_tuple(std::get<1>(sensor_data), std::get<2>(sensor_data));
  return rpyt_velocity_controller_.isConverged(vel_sensor_data);
}

VelocityYawRate JoystickVelocityController::convertJoystickToVelocityYawRate(
    const Joystick joy_sensor_data) {
  VelocityYawRate velocity_yaw_rate;
  velocity_yaw_rate.x =
      math::map(joy_sensor_data.channel1,
                -joystick_velocity_controller_config_.max_channel1(),
                joystick_velocity_controller_config_.max_channel1(),
                -joystick_velocity_controller_config_.max_velocity(),
                joystick_velocity_controller_config_.max_velocity());

  velocity_yaw_rate.y =
      math::map(joy_sensor_data.channel2,
                -joystick_velocity_controller_config_.max_channel2(),
                joystick_velocity_controller_config_.max_channel2(),
                -joystick_velocity_controller_config_.max_velocity(),
                joystick_velocity_controller_config_.max_velocity());

  velocity_yaw_rate.z =
      math::map(joy_sensor_data.channel3,
                -joystick_velocity_controller_config_.max_channel3(),
                joystick_velocity_controller_config_.max_channel3(),
                -joystick_velocity_controller_config_.max_velocity(),
                joystick_velocity_controller_config_.max_velocity());

  // Positive stick movement --> Clockwise rotation. Hence, negative sign
  velocity_yaw_rate.yaw_rate =
      math::map(-joy_sensor_data.channel4,
                -joystick_velocity_controller_config_.max_channel4(),
                joystick_velocity_controller_config_.max_channel4(),
                -joystick_velocity_controller_config_.max_yaw_rate(),
                joystick_velocity_controller_config_.max_yaw_rate());
  return velocity_yaw_rate;
}
