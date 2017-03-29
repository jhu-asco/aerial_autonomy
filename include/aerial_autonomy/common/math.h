#pragma once

namespace math {

/**
 * @brief Wrap an angle to be in the range [-pi, pi)
 * @param x Angle to wrap
 * @return Wrapped angle
 */
static double angleWrap(double x) {
  x = fmod(x + M_PI, 2 * M_PI);
  if (x < 0)
    x += 2 * M_PI;
  return x - M_PI;
}
}
