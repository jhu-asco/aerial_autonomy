#pragma once
struct Position {
  Position() : x(0), y(0), z(0) {}
  Position(double x, double y, double z) : x(x), y(y), z(z) {}
  double x;
  double y;
  double z;
};
