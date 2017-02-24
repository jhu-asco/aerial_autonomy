#pragma once
struct Velocity {
  Velocity() : x(0), y(0), z(0) {}
  Velocity(double x, double y, double z) : x(x), y(y), z(z) {}
  double x;
  double y;
  double z;
  bool operator==(const Velocity &v) const {
    return (x == v.x && y == v.y && z == v.z);
  }
  bool operator!=(const Velocity &v) const { return !(*this == v); }
};
