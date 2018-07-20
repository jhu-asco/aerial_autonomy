#pragma once

/**
* @brief Define Math constants such as pi, e etc.
*/
#define _USE_MATH_DEFINES
#include <Eigen/Dense>
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

/**
 * @brief The hat operator. Computes a skew symmetric matrix from a vector
 * @param v The vector to convert to a skew-symmetric matrix
 * @return The skew-symmetric matric
 */
Eigen::Matrix3d hat(const Eigen::Vector3d &v);

/**
  * @brief Solve the Sylvester equation AX + XB + C = 0
  * @param A A
  * @param B B
  * @param C C
  * @return X
  */
Eigen::MatrixXd sylvester(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                          const Eigen::MatrixXd &C);

/**
  * @brief Cumulative sum of a vector
  * @param vec_eigen Eigen::VectorXd
  * @return Cumulative sum of a vector
  */
Eigen::VectorXd cumsumEigen(const Eigen::VectorXd &vec_eigen);
}
