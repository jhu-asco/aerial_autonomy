#pragma once
#include <cmath>
/**
* @brief Stores 3D acceleration
*/
struct Acceleration {
  /**
  * @brief Implicit Constructor
  */
  Acceleration() : x(0), y(0), z(0) {}
  /**
  * @brief Explicit constructor
  */
  Acceleration(double x, double y, double z) : x(x), y(y), z(z) {}
  double x; ///< x component in m
  double y; ///< y component in m
  double z; ///< z component in m
  /**
* @brief Returns the norm of the vector
* @return the norm
*/
  double norm() const {
    return std::sqrt(std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2));
  }

  /**
   * @brief Compare two accelerations
   *
   * @param a Acceleration to compare against
   *
   * @return True if two accelerations are same
   */
  bool operator==(const Acceleration &a) const {
    return (x == a.x && y == a.y && z == a.z);
  }
  /**
  * @brief Compare two accelerations
  *
  * @param a Acceleration to compare against
  *
  * @return True if two accelerations are not same
  */
  bool operator!=(const Acceleration &a) const { return !(*this == a); }
  /**
  * @ brief Add two accelerations
  * @param a Acceleration to add
  * @return Sum of this acceleration and a
  */
  Acceleration operator+(const Acceleration &a) const {
    return Acceleration(a.x + this->x, a.y + this->y, a.z + this->z);
  }
  /**
  * @ brief Subtract two accelerations
  * @param a Acceleration to subtract
  * @return Difference of this acceleration and a
  */
  Acceleration operator-(const Acceleration &a) const {
    return Acceleration(this->x - a.x, this->y - a.y, this->z - a.z);
  }

  /**
  * @brief Multiply times a scalar
  * @param m Multiplier
  * @return Scaled acceleration
  */
  Acceleration operator*(const double &m) const {
    return Acceleration(this->x * m, this->y * m, this->z * m);
  }
  /**
  * @brief Divide by a scalar
  * @param m Divisor
  * @return Scaled acceleration
  */
  Acceleration operator/(const double &m) const {
    return this->operator*(1. / m);
  }
};
