#pragma once
#include <cmath>
/**
* @brief Stores 3D snap
* (4th derivative of position)
*/
struct Snap {
  /**
  * @brief Implicit Constructor
  */
  Snap() : x(0), y(0), z(0) {}
  /**
  * @brief Explicit constructor
  */
  Snap(double x, double y, double z) : x(x), y(y), z(z) {}
  double x; ///< x component in m/s^4
  double y; ///< y component in m/s^4
  double z; ///< z component in m/s^4

  /**
   * @brief Check if two snap vectors are the same
   *
   * @param s The vector against which the current vector is compared.
   *
   * @return True if vectors are equal
   */
  bool operator==(const Snap &s) const {
    return (x == s.x && y == s.y && z == s.z);
  }
  /**
  * @brief Check if two vectors are not equal
  *
  * @param s The vector against which the current vector is compared.
  *
  * @return  True if two vectors are not equal
  */
  bool operator!=(const Snap &s) const { return !(*this == s); }
  /**
  * @brief Add two  entities
  *
  * @param s Snap to be added to current snap
  *
  * @return Sum snaps
  */
  Snap operator+(const Snap &s) const {
    return Snap(this->x + s.x, this->y + s.y, this->z + s.z);
  }
  /**
  * @brief Substract two  entities
  *
  * @param s Snap to be substracted from current snap
  *
  * @return Difference between snaps
  */
  Snap operator-(const Snap &s) const {
    return Snap(this->x - s.x, this->y - s.y, this->z - s.z);
  }
};
