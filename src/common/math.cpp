#include "aerial_autonomy/common/math.h"

#include <algorithm>

namespace math {

double angleWrap(double x) {
  x = std::fmod(x + M_PI, 2 * M_PI);
  if (x < 0)
    x += 2 * M_PI;
  return x - M_PI;
}

double clamp(double x, double min, double max) {
  return std::min(std::max(x, min), max);
}
}
