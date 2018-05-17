#pragma once

#include "aerial_autonomy/types/acceleration.h"
#include "aerial_autonomy/types/jerk.h"
#include "aerial_autonomy/types/position.h"
#include "aerial_autonomy/types/velocity.h"

struct ParticleState {
  Position p;
  Velocity v;
  Acceleration a;
  Jerk j;
};
