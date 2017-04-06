#pragma once
#include "aerial_autonomy/types/velocity.h"
#include <aerial_autonomy/common/math.h>

/**
* @brief Store velocity and yaw
* Used as goal for builtin velocity and yaw controller
* for UAV system
*/
struct VelocityYaw : public Velocity {
  /**
  * @brief Implicit constructor
  * Instantiates velocity and yaw to zero
  */
  VelocityYaw() : Velocity(), yaw(0) {}
  /**
  * @brief Explicit constructor
  *
  * @param v Velocity vector
  * @param yaw Orientation around body axis (rad)
  */
  VelocityYaw(Velocity v, double yaw) : Velocity(v), yaw(yaw) {}
  /**
  * @brief Explicit constructor
  *
  * @param x x component (m/s)
  * @param y y component (m/s)
  * @param z z component (m/s)
  * @param yaw Orientation around global z axis (rad)
  */
  VelocityYaw(double x, double y, double z, double yaw)
      : Velocity(x, y, z), yaw(yaw) {}
  double yaw; ///< Orientation around global z axis

  /**
   * @brief Compare two velocityyaws
   *
   * @param v VelocityYaw to compare against
   *
   * @return True if same
   */
  bool operator==(const VelocityYaw &v) const {
    return (Velocity::operator==(v) && yaw == v.yaw);
  }
  /**
  * @brief Compare two velocityyaws
  *
  * @param v VelocityYaw to compare against
  *
  * @return True if not same
  */
  bool operator!=(const VelocityYaw &v) const { return !(*this == v); }

  /**
  * @brief Substract two velocity yaw entities
  *
  * @param v VelocityYaw to be substracted from current velocity yaw
  *
  * @return Difference between velocity yaws
  */
  VelocityYaw operator-(const VelocityYaw &v) const {
    return VelocityYaw(this->x - v.x, this->y - v.y, this->z - v.z,
                       math::angleWrap(this->yaw - v.yaw));
  }
};
