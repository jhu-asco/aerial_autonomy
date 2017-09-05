#include "aerial_autonomy/common/math.h"

#include <algorithm>

/**
* @brief Math namespace to separate math functions from
* system functions if exist
*/
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

double map(double input, double input_min, double input_max, double output_min,
           double output_max) {
  if (input > input_max)
    return output_max;

  else if (input < input_min)
    return output_min;

  return (output_min +
          ((input - input_min) * (output_max - output_min)) /
              (input_max - input_min));
}
}
