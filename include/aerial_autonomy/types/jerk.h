#pragma once
#include <cmath>
/**
* @brief Stores 3D jerk
* (3rd derivative of position)
*/
struct Jerk {
  /**
  * @brief Implicit Constructor
  */
  Jerk() : x(0), y(0), z(0) {}
  /**
  * @brief Explicit constructor
  */
  Jerk(double x, double y, double z) : x(x), y(y), z(z) {}
  double x; ///< x component in m
  double y; ///< y component in m
  double z; ///< z component in m

  /**
   * @brief Check if two jerk vectors are the same
   *
   * @param j The vector against which the current vector is compared.
   *
   * @return True if vectors are equal
   */
  bool operator==(const Jerk &j) const {
    return (x == j.x && y == j.y && z == j.z);
  }
  /**
  * @brief Check if two vectors are not equal
  *
  * @param j The vector against which the current vector is compared.
  *
  * @return  True if two vectors are not equal
  */
  bool operator!=(const Jerk &j) const { return !(*this == j); }
  /**
  * @brief Add two  entities
  *
  * @param j Jerk to be added to current jerk
  *
  * @return Sum jerks
  */
  Jerk operator+(const Jerk &j) const {
    return Jerk(this->x + j.x, this->y + j.y, this->z + j.z);
  }
  /**
  * @brief Substract two  entities
  *
  * @param j Jerk to be substracted from current jerk
  *
  * @return Difference between jerks
  */
  Jerk operator-(const Jerk &j) const {
    return Jerk(this->x - j.x, this->y - j.y, this->z - j.z);
  }
};