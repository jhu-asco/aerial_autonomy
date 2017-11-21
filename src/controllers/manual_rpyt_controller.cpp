#include "aerial_autonomy/controllers/manual_rpyt_controller.h"
#include "aerial_autonomy/common/math.h"

bool ManualRPYTController::runImplementation(JoystickYaw sensor_data,
                                             EmptyGoal goal,
                                             RollPitchYawThrust &control) {
  /// \todo(matt): need to pass RC mapping as parameter
  control.r =
      math::map(sensor_data.channel1, -10000, 10000, -M_PI / 6, M_PI / 6);
  control.p =
      math::map(sensor_data.channel2, -10000, 10000, -M_PI / 6, M_PI / 6);
  control.t = math::map(sensor_data.channel3, -10000, 10000, 10, 100);

  double yaw_rate = math::map(sensor_data.channel4, -10000, 10000, -M_PI, M_PI);
  /// \todo(matt): need to pass in frequency as a parameter
  control.y = sensor_data.yaw - yaw_rate * 0.02;
  if (control.y > M_PI)
    control.y = control.y - 2 * M_PI;
  else if (control.y < -M_PI)
    control.y = control.y + 2 * M_PI;

  return true;
}
