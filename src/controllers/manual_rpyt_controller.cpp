#include "aerial_autonomy/controllers/manual_rpyt_controller.h"

RollPitchYawThrust
ManualRPYTController::runImplementation(JoysticksYaw sensor_data,
                                        EmptyGoal goal, ControllerStatus &) {
  RollPitchYawThrust controls;
  /// \todo(matt): need to pass RC mapping as parameter
  controls.r = map(sensor_data.channel1, -10000, 10000, -M_PI / 6, M_PI / 6);
  controls.p = map(sensor_data.channel2, -10000, 10000, -M_PI / 6, M_PI / 6);
  controls.t = map(sensor_data.channel3, -10000, 10000, 10, 100);

  double yaw_rate = map(sensor_data.channel4, -10000, 10000, -M_PI, M_PI);
  /// \todo(matt): need to pass in frequency as a parameter
  controls.y = sensor_data.yaw - yaw_rate * 0.02;
  if (controls.y > M_PI)
    controls.y = controls.y - 2 * M_PI;
  else if (controls.y < -M_PI)
    controls.y = controls.y + 2 * M_PI;

  return controls;
}

double ManualRPYTController::map(double input, double input_min,
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
