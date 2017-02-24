#pragma once
#include "aerial_autonomy/types/position.h"

struct PositionYaw : public Position {
  PositionYaw() : Position(), yaw(0) {}
  PositionYaw(Position p, double yaw) : Position(p), yaw(yaw) {}
  PositionYaw(double x, double y, double z, double yaw)
      : Position(x, y, z), yaw(yaw) {}
  double yaw;
  bool operator==(const PositionYaw &p) const {
    return (Position::operator==(p) && yaw == p.yaw);
  }
  bool operator!=(const PositionYaw &p) const { return !(*this == p); }
};
