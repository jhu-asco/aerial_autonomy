#include "aerial_autonomy/controllers/manual_rpyt_controller.h"
#include "aerial_autonomy/common/math.h"

bool ManualRPYTController::runImplementation(JoystickYaw sensor_data,
                                             EmptyGoal goal,
                                             RollPitchYawThrust &control) {

  control.r = math::map(sensor_data.channel1, -config_.max_channel1(),
                        config_.max_channel1(), -config_.max_roll(),
                        config_.max_roll());
  control.p = math::map(sensor_data.channel2, -config_.max_channel2(),
                        config_.max_channel2(), -config_.max_pitch(),
                        config_.max_pitch());
  control.t = math::map(sensor_data.channel3, -config_.max_channel3(),
                        config_.max_channel3(), config_.min_thrust(),
                        config_.max_thrust());

  double yaw_rate = math::map(sensor_data.channel4, -config_.max_channel4(),
                              config_.max_channel4(), -config_.max_yaw_rate(),
                              config_.max_yaw_rate());

  control.y = math::angleWrap(last_yaw_ - yaw_rate * controller_timer_duration_);
  last_yaw_ = control.y;
  return true;
}
