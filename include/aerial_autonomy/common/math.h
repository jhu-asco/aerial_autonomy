#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

namespace math {

/**
 * @brief Wrap an angle to be in the range [-pi, pi)
 * @param x Angle to wrap
 * @return Wrapped angle
 */
double angleWrap(double x);

/**
 * @brief Clip a number to bewteen a min and max value
 * @param x Number to clamp
 * @param min Minimum value
 * @param max Maximum value
 * @return Clamped value
 */
double clamp(double x, double min, double max);
}
