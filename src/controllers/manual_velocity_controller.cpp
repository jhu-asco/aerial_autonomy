#include "aerial_autonomy/controllers/manual_velocity_controller.h"
#include "aerial_autonomy/common/math.h"

bool ManualVelocityController::runImplementation(
    std::tuple<Joysticks, VelocityYaw> sensor_data, EmptyGoal goal,
    RollPitchYawThrust &control) {

  VelocityYaw vel_goal;
  Joysticks joy_sensor_data = std::get<0>(sensor_data);

  vel_goal.x = map(joy_sensor_data.channel1,
                   -manual_velocity_controller_config_.max_channel1(),
                   manual_velocity_controller_config_.max_channel1(),
                   -manual_velocity_controller_config_.max_velocity(),
                   manual_velocity_controller_config_.max_velocity());

  vel_goal.y = map(joy_sensor_data.channel2,
                   -manual_velocity_controller_config_.max_channel2(),
                   manual_velocity_controller_config_.max_channel2(),
                   -manual_velocity_controller_config_.max_velocity(),
                   manual_velocity_controller_config_.max_velocity());

  vel_goal.z = map(joy_sensor_data.channel3,
                   -manual_velocity_controller_config_.max_channel3(),
                   manual_velocity_controller_config_.max_channel3(),
                   -manual_velocity_controller_config_.max_velocity(),
                   manual_velocity_controller_config_.max_velocity());

  double yaw_rate = map(joy_sensor_data.channel4,
                        -manual_velocity_controller_config_.max_channel4(),
                        manual_velocity_controller_config_.max_channel4(),
                        -manual_velocity_controller_config_.max_velocity(),
                        manual_velocity_controller_config_.max_velocity());

  double dt = 1.0 / manual_velocity_controller_config_.controller_frequency();
  VelocityYaw vel_sensor_data = std::get<1>(sensor_data);
  vel_goal.yaw = math::angleWrap(vel_sensor_data.yaw - yaw_rate * dt);

  rpyt_velocity_controller_.setGoal(vel_goal);
  rpyt_velocity_controller_.run(vel_sensor_data, control);

  return true;
}

double ManualVelocityController::map(double input, double input_min,
                                     double input_max, double output_min,
                                     double output_max) {
  if (input > input_max)
    return output_max;
  else if (input < input_min)
    return output_min;
  return output_min +
         ((input - input_min) * (output_max - output_min)) /
             (input_max - input_min);
}