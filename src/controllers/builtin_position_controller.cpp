#include "aerial_autonomy/controllers/builtin_position_controller.h"

PositionYaw BuiltInPositionController::runImplementation(EmptySensor,
                                                         PositionYaw goal) {
  return goal;
}
