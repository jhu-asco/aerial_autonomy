#include "aerial_autonomy/controllers/builtin_position_controller.h"

PositionYaw BuiltInPositionController::runImpl(NoSensor sensor_data, PositionYaw goal) {
  return goal;
}
