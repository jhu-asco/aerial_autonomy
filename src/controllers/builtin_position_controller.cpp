#include "aerial_autonomy/controllers/builtin_position_controller.h"

PositionYaw BuiltInPositionController::runImpl(EmptySensor sensor_data, PositionYaw goal) {
  return goal;
}
