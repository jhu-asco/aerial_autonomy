#include "aerial_autonomy/controllers/manual_rpyt_controller.h"
#include "aerial_autonomy/common/math.h"

bool ManualRPYTController::runImplementation(Joystick sensor_data,
                                             EmptyGoal goal,
                                             RollPitchYawRateThrust &control) {
  /// \todo(matt): need to pass RC mapping as parameter
  control.r =
      math::map(sensor_data.channel1, -10000, 10000, -M_PI / 6, M_PI / 6);
  control.p =
      math::map(sensor_data.channel2, -10000, 10000, -M_PI / 6, M_PI / 6);
  control.t = math::map(sensor_data.channel3, -10000, 10000, 10, 100);

  control.y = math::map(sensor_data.channel4, -10000, 10000, -M_PI, M_PI);

  return true;
}
