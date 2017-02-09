#include "aerial_autonomy/controllers/builtin_position_controller.h"

PositionYaw BuiltInPositionController::runImpl(EmptySensor, PositionYaw goal) {
  return goal;
}
