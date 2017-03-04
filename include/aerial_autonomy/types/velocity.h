#pragma once
/**
* @brief Store velocity vector
*/
struct Velocity {
  /**
  * @brief Constructor with implicit instantiation of
  * velocity vector to zero
  */
  Velocity() : x(0), y(0), z(0) {}
  /**
  * @brief Constructor with explicit instantiation
  *
  * @param x x component (m/s)
  * @param y y component (m/s)
  * @param z z component (m/s)
  */
  Velocity(double x, double y, double z) : x(x), y(y), z(z) {}
  double x;///< x component in m/s
  double y;///< y component in m/s
  double z;///< z component in m/s
  /**
  * @brief Check if two velocity vectors are the same
  *
  * @param v The vector agains which the current vector is compared.
  *
  * @return True if vectors are equal
  */
  bool operator==(const Velocity &v) const {
    return (x == v.x && y == v.y && z == v.z);
  }
  /**
  * @brief Check if two vectors are not equal
  *
  * @param v The vector agains which the current vector is compared.
  *
  * @return  True if two vectors are not equal
  */
  bool operator!=(const Velocity &v) const { return !(*this == v); }
};
