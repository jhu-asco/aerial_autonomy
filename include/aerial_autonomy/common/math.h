#pragma once

/**
* @brief Define Math constants such as pi, e etc.
*/
#define _USE_MATH_DEFINES
#include <cmath>
#include <stdexcept>
#include <tf/tf.h>

/**
* @brief Math namespace to separate math functions from
* system functions if exist
*/
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

/**
* @brief Generic map function to map input range to output range
*
* @param input Input value to std::map<key, value> map;
* @param input_min Min for input
* @param input_max Max for input
* @param output_min Min for output
* @param output_max Max for output
*
* @return Map the input based on input range to output in output range
*/
double map(double input, double input_min, double input_max, double output_min,
           double output_max);
}
