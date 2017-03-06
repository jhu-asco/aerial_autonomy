#pragma once
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
  double x;///< x component in m
  double y;///< y component in m
  double z;///< z component in m
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
};
