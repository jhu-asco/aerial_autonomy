#pragma once
struct RollPitchYawThrust {
  RollPitchYawThrust() : r(0), p(0), y(0), t(0) {}
  RollPitchYawThrust(double r, double p, double y, double t)
      : r(r), p(p), y(y), t(t) {}
  double r;
  double p;
  double y;
  double t;
};
