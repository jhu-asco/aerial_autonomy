#include "aerial_autonomy/controllers/builtin_position_controller.h"

PositionYaw BuiltInPositionController::run(NoSensor sensor_data) {
  return goal_;
}
