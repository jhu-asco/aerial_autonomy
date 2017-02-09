#include "aerial_autonomy/controllers/builtin_velocity_controller.h"

VelocityYaw BuiltInVelocityController::runImplementation(EmptySensor,
                                                         VelocityYaw goal) {
  return goal;
}
