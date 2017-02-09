#pragma once
struct Velocity {
  Velocity() : x(0), y(0), z(0) {}
  Velocity(double x, double y, double z) : x(x), y(y), z(z) {}
  double x;
  double y;
  double z;
};
