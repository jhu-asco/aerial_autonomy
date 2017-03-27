#pragma once
#include "aerial_autonomy/types/position.h"

/**
* @brief Stores Position, yaw.
* PositionYaw is used as the goal for
* UAV systems.
*/
struct PositionYaw : public Position {
  /**
  * @brief Implicit constructor
  *
  * Instantiate position and yaw to zero
  */
  PositionYaw() : Position(), yaw(0) {}
  /**
  * @brief Explicit constructor that takes in position, yaw
  *
  * @param p Position
  * @param yaw orientation about body axis used as goal for quadrotor.
  */
  PositionYaw(Position p, double yaw) : Position(p), yaw(yaw) {}
  /**
  * @brief Explicit constructor with x,y,z, and yaw
  *
  * @param x x component (m)
  * @param y y component (m)
  * @param z z component (m)
  * @param yaw yaw component (rad)
  */
  PositionYaw(double x, double y, double z, double yaw)
      : Position(x, y, z), yaw(yaw) {}
  double yaw; ///< Orientation about body axis rad

  /**
   * @brief Compare two position and yaw entities
   *
   * @param p PositionYaw to compare against
   *
   * @return True if position and yaw are the same
   */
  bool operator==(const PositionYaw &p) const {
    return (Position::operator==(p) && yaw == p.yaw);
  }
  /**
  * @brief Compare two position and yaw entities
  *
  * @param p PositionYaw to add
  *
  * @return True if position and yaw are different
  */
  bool operator!=(const PositionYaw &p) const { return !(*this == p); }
  /**
  * @brief Add two position and yaw entities
  * @param p PositionYaw to add
  * @return Sum of position and yaw
  */
  PositionYaw operator+(const PositionYaw &p) const {
    Position p1(this->x, this->y, this->z);
    Position p2(p.x, p.y, p.z);
    return PositionYaw(p1 + p2, this->yaw + p.yaw);
  }
  /**
  * @brief Subtract two position and yaw entities
  * @param p PositionYaw to subtract
  * @return Difference of position and yaw
  */
  PositionYaw operator-(const PositionYaw &p) const {
    Position p1(this->x, this->y, this->z);
    Position p2(p.x, p.y, p.z);
    return PositionYaw(p1 - p2, this->yaw - p.yaw);
  }
};
