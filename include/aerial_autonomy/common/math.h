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
* @brief Generate a tf transform from a vector of xyzrpy
*
* @tparam T The vector type can be Eigen, std vector, protobuf
* @param input The input vector containing x,y,z, r,p,y
*
* @return tf transform
*/
template <class T> tf::Transform getTransformFromVector(const T &input) {
  tf::Transform transform;
  if (input.size() != 6) {
    throw std::runtime_error("The input does not have 6 elements x,y,z, r,p,y");
  } else {
    transform.setOrigin(tf::Vector3(input[0], input[1], input[2]));
    transform.setRotation(
        tf::createQuaternionFromRPY(input[3], input[4], input[5]));
  }
  return transform;
}
}
