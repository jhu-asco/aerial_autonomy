#pragma once
#include "aerial_autonomy/types/velocity.h"

/**
* @brief Store velocity and yaw rate
*/
struct VelocityYawRate : public Velocity {
  /**
  * @brief Implicit constructor
  * Instantiates velocity and yaw rate to zero
  */
  VelocityYawRate() : Velocity(), yaw_rate(0) {}
  /**
  * @brief Explicit constructor
  *
  * @param v Velocity vector
  * @param yaw_rate Angular rate around global z (rad/s)
  */
  VelocityYawRate(Velocity v, double yaw_rate)
      : Velocity(v), yaw_rate(yaw_rate) {}
  /**
  * @brief Explicit constructor
  *
  * @param x x component (m/s)
  * @param y y component (m/s)
  * @param z z component (m/s)
  * @param yaw Angular rate around global z axis (rad/s)
  */
  VelocityYawRate(double x, double y, double z, double yaw_rate)
      : Velocity(x, y, z), yaw_rate(yaw_rate) {}
  double yaw_rate; ///< Angular rate around global z axis

  /**
   * @brief Compare two VelocityYawRate
   *
   * @param v VelocityYawRate to compare against
   *
   * @return True if same
   */
  bool operator==(const VelocityYawRate &v) const {
    return (Velocity::operator==(v) && yaw_rate == v.yaw_rate);
  }
  /**
  * @brief Compare two VelocityYawRate
  *
  * @param v VelocityYawRate to compare against
  *
  * @return True if not same
  */
  bool operator!=(const VelocityYawRate &v) const { return !(*this == v); }
};
