#include "aerial_autonomy/controllers/joystick_velocity_controller.h"
#include "aerial_autonomy/common/math.h"

bool JoystickVelocityController::runImplementation(
    std::tuple<Joystick, VelocityYaw> sensor_data, EmptyGoal goal,
    RollPitchYawThrust &control) {

  VelocityYaw vel_goal;
  Joystick joy_sensor_data = std::get<0>(sensor_data);

  vel_goal.x = math::map(joy_sensor_data.channel1,
                         -joystick_velocity_controller_config_.max_channel1(),
                         joystick_velocity_controller_config_.max_channel1(),
                         -joystick_velocity_controller_config_.max_velocity(),
                         joystick_velocity_controller_config_.max_velocity());

  vel_goal.y = math::map(joy_sensor_data.channel2,
                         -joystick_velocity_controller_config_.max_channel2(),
                         joystick_velocity_controller_config_.max_channel2(),
                         -joystick_velocity_controller_config_.max_velocity(),
                         joystick_velocity_controller_config_.max_velocity());

  vel_goal.z = math::map(joy_sensor_data.channel3,
                         -joystick_velocity_controller_config_.max_channel3(),
                         joystick_velocity_controller_config_.max_channel3(),
                         -joystick_velocity_controller_config_.max_velocity(),
                         joystick_velocity_controller_config_.max_velocity());

  // Positive stick movement --> Clockwise rotation. Hence, negative sign
  double yaw_rate =
      math::map(-joy_sensor_data.channel4,
                -joystick_velocity_controller_config_.max_channel4(),
                joystick_velocity_controller_config_.max_channel4(),
                -joystick_velocity_controller_config_.max_yaw_rate(),
                joystick_velocity_controller_config_.max_yaw_rate());

  VelocityYaw vel_sensor_data = std::get<1>(sensor_data);
  vel_goal.yaw =
      math::angleWrap(last_yaw_ + yaw_rate * controller_timer_duration_);

  last_yaw_ = vel_goal.yaw;
  rpyt_velocity_controller_.setGoal(vel_goal);
  rpyt_velocity_controller_.run(vel_sensor_data, control);

  return true;
}

ControllerStatus JoystickVelocityController::isConvergedImplementation(
    std::tuple<Joystick, VelocityYaw> sensor_data, EmptyGoal goal) {
  Joystick joy_data = std::get<0>(sensor_data);
  VelocityYaw velocity_data = std::get<1>(sensor_data);

  DATA_LOG("joystick_velocity_controller")
      << joy_data.channel1 << joy_data.channel2 << joy_data.channel3
      << joy_data.channel4 << velocity_data.x << velocity_data.y
      << velocity_data.z << velocity_data.yaw << DataStream::endl;

  return rpyt_velocity_controller_.isConverged(velocity_data);
}
