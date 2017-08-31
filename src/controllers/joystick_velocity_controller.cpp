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

  double yaw_rate =
      math::map(joy_sensor_data.channel4,
                -joystick_velocity_controller_config_.max_channel4(),
                joystick_velocity_controller_config_.max_channel4(),
                -joystick_velocity_controller_config_.max_velocity(),
                joystick_velocity_controller_config_.max_velocity());

  double dt = controller_timer_duration_ / 1000.0;
  VelocityYaw vel_sensor_data = std::get<1>(sensor_data);
  vel_goal.yaw = math::angleWrap(vel_sensor_data.yaw - yaw_rate * dt);

  rpyt_velocity_controller_.setGoal(vel_goal);
  rpyt_velocity_controller_.run(vel_sensor_data, control);

  return true;
}