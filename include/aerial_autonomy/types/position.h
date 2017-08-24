#pragma once

#include <cmath>

/**
* @brief Store 3D position
*/
struct Position {
  /**
  * @brief Implicit constructor
  * Instantiate position to (0,0,0)m
  */
  Position() : x(0), y(0), z(0) {}
  /**
  * @brief Explicit constructor
  *
  * @param x x component in m
  * @param y y component in m
  * @param z z component in m
  */
  Position(double x, double y, double z) : x(x), y(y), z(z) {}
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
   * @brief Compare two positions
   *
   * @param p Position to compare against
   *
   * @return True if two positions are same
   */
  bool operator==(const Position &p) const {
    return (x == p.x && y == p.y && z == p.z);
  }
  /**
  * @brief Compare two positions
  *
  * @param p Position to compare against
  *
  * @return True if two positions are not same
  */
  bool operator!=(const Position &p) const { return !(*this == p); }
  /**
  * @ brief Add two positions
  * @param p Position to add
  * @return Sum of this position and p
  */
  Position operator+(const Position &p) const {
    return Position(p.x + this->x, p.y + this->y, p.z + this->z);
  }
  /**
  * @ brief Subtract two positions
  * @param p Position to subtract
  * @return Difference of this position and p
  */
  Position operator-(const Position &p) const {
    return Position(this->x - p.x, this->y - p.y, this->z - p.z);
  }

  /**
  * @brief Multiply times a scalar
  * @param m Multiplier
  * @return Scaled position
  */
  Position operator*(const double &m) const {
    return Position(this->x * m, this->y * m, this->z * m);
  }
  /**
  * @brief Divide by a scalar
  * @param m Divisor
  * @return Scaled position
  */
  Position operator/(const double &m) const { return this->operator*(1. / m); }
};
