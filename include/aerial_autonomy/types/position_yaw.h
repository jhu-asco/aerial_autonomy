#pragma once
#include "aerial_autonomy/types/position.h"
#include "aerial_autonomy/types/velocity_yaw_rate.h"
#include <aerial_autonomy/common/math.h>

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

  /**
   * @brief Simplified constructor with x, x, x and yaw
   *
   * @param position_value position component (m)
   * @param yaw yaw component (rad)
   */
  PositionYaw(double position_value, double yaw)
      : Position(position_value, position_value, position_value), yaw(yaw) {}

  /**
  * @brief Return position
  * @return position
  */
  Position position() { return Position(x, y, z); }

  /**
  * @brief Set the Position
  * @param p Position to set
  */
  void setPosition(const Position &p) {
    x = p.x;
    y = p.y;
    z = p.z;
  }

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
    return PositionYaw(p1 + p2, math::angleWrap(this->yaw + p.yaw));
  }

  /**
   * @brief Add velocity yaw rate to position
   *
   * @param p velocity yaw rate to add
   *
   * @return  updated position yaw
   */
  PositionYaw operator+(const VelocityYawRate &p) const {
    return PositionYaw(x + p.x, y + p.y, z + p.z,
                       math::angleWrap(yaw + p.yaw_rate));
  }

  /**
  * @brief Subtract two position and yaw entities
  * @param p PositionYaw to subtract
  * @return Difference of position and yaw
  */
  PositionYaw operator-(const PositionYaw &p) const {
    Position p1(this->x, this->y, this->z);
    Position p2(p.x, p.y, p.z);
    return PositionYaw(p1 - p2, math::angleWrap(this->yaw - p.yaw));
  }

  /**
  * @brief Multiply times a scalar
  * @param m Multiplier
  * @return Scaled position yaw
  */
  PositionYaw operator*(const double &m) const {
    return PositionYaw(this->x * m, this->y * m, this->z * m, this->yaw * m);
  }

  /**
   * @brief create a position from x, y, z members
   *
   * @return created position
   */
  Position position() const { return Position(x, y, z); }

  /**
   * @brief Clamp the position yaw between min and max
   *
   * @param min Minimum position yaw
   * @param max Maximum position yaw
   *
   */
  void clamp(const PositionYaw &min, const PositionYaw &max) {
    x = math::clamp(x, min.x, max.x);
    y = math::clamp(y, min.y, max.y);
    z = math::clamp(z, min.z, max.z);
    yaw = math::clamp(yaw, min.yaw, max.yaw);
  }

  void clamp(const PositionYaw &max) {
    PositionYaw min = max * -1.0;
    clamp(min, max);
  }
};
