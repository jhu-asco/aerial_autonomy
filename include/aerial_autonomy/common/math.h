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

/**
* @brief Generate a vector of tf transforms from a vector of xyzrpy
*
* @tparam T The vector type can be Eigen, std vector, protobuf
* @param input The input vector containing x,y,z, r,p,y
*
* @return vector of tf::Transform
*
*/

/**
* \todo (Matt) Add proto for transform and just process a list of the
* proto transforms here
*/
template <class T>
std::vector<tf::Transform> getTransformsFromVector(const T &input) {
  if (input.size() % 6 != 0) {
    throw std::runtime_error(
        "The input does not have a multiple of 6 elements x,y,z, r,p,y");
  }

  std::vector<tf::Transform> transforms(input.size() / 6);
  for (int i = 0, j = 0; i < input.size(); i += 6, j++) {
    transforms.at(j).setOrigin(
        tf::Vector3(input[i + 0], input[i + 1], input[i + 2]));
    transforms.at(j).setRotation(
        tf::createQuaternionFromRPY(input[i + 3], input[i + 4], input[i + 5]));
  }
  return transforms;
}
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