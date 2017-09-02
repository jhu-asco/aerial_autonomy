#include "aerial_autonomy/controllers/manual_rpyt_controller.h"

bool ManualRPYTController::runImplementation(JoystickYaw sensor_data,
                                             EmptyGoal goal,
                                             RollPitchYawThrust &control) {
  /// \todo(matt): need to pass RC mapping as parameter
  control.r = map(sensor_data.channel1, -10000, 10000, -M_PI / 6, M_PI / 6);
  control.p = map(sensor_data.channel2, -10000, 10000, -M_PI / 6, M_PI / 6);
  control.t = map(sensor_data.channel3, -10000, 10000, 10, 100);

  double yaw_rate = map(sensor_data.channel4, -10000, 10000, -M_PI, M_PI);
  /// \todo(matt): need to pass in frequency as a parameter
  control.y = sensor_data.yaw - yaw_rate * 0.02;
  if (control.y > M_PI)
    control.y = control.y - 2 * M_PI;
  else if (control.y < -M_PI)
    control.y = control.y + 2 * M_PI;

  return true;
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
