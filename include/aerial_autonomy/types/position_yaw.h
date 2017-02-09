#pragma once
#include "aerial_autonomy/types/position.h"

struct PositionYaw : public Position {
  PositionYaw() : Position(), yaw(0) {}
  PositionYaw(double x, double y, double z, double yaw)
      : Position(x, y, z), yaw(yaw) {}
  double yaw;
};
