#pragma once
#include "aerial_autonomy/types/velocity.h"

struct VelocityYaw : public Velocity {
  VelocityYaw() : Velocity(), yaw(0) {}
  VelocityYaw(double x, double y, double z, double yaw)
      : Velocity(x, y, z), yaw(yaw) {}
  double yaw;
};
