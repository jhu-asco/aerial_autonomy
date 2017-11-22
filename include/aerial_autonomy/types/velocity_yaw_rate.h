#pragma once
#include "aerial_autonomy/types/velocity.h"
#include <aerial_autonomy/common/math.h>

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
  /**
  * @brief Add two velocity and yaw rate entities
  * @param p VelocityYawRate to add
  * @return Sum of velocity and yaw rate
  */
  VelocityYawRate operator+(const VelocityYawRate &p) const {
    return VelocityYawRate(this->x + p.x, this->y + p.y, this->z + p.z,
                           math::angleWrap(this->yaw_rate + p.yaw_rate));
  }
  /**
  * @brief Multiply times a scalar
  * @param m Multiplier
  * @return Scaled velocity yaw rate
  */
  VelocityYawRate operator*(const double &m) const {
    return VelocityYawRate(this->x * m, this->y * m, this->z * m,
                           math::angleWrap(this->yaw_rate * m));
  }
  /**
  * @brief Substract two velocity yaw rate entities
  *
  * @param v VelocityYawRate to be substracted from current velocity yaw
  *
  * @return Difference between velocity yaw rates
  */
  VelocityYawRate operator-(const VelocityYawRate &v) const {
    return VelocityYawRate(this->x - v.x, this->y - v.y, this->z - v.z,
                           this->yaw_rate - v.yaw_rate);
  }
};
