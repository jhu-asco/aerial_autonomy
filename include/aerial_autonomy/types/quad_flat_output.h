#pragma once
#include "aerial_autonomy/types/acceleration.h"
#include "aerial_autonomy/types/jerk.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/snap.h"
#include "aerial_autonomy/types/velocity_yaw_rate.h"
/**
* @brief Store flat output state
* of a quad
*/
struct QuadFlatOutput {
  /**
  * @brief Implicit Constructor
  */
  QuadFlatOutput() {}
  /**
  * @brief Constructor specifying position
  */
  QuadFlatOutput(PositionYaw position_yaw) : p(position_yaw) {}

  /**
  * @brief Explicit Constructor
  */
  QuadFlatOutput(PositionYaw position_yaw, VelocityYawRate velocity_yaw_rate,
                 Acceleration acceleration, Jerk jerk, Snap snap,
                 double yaw_acceleration)
      : p(position_yaw), v(velocity_yaw_rate), a(acceleration), j(jerk),
        s(snap), ga2(yaw_acceleration) {}

  PositionYaw p;     ///< position
  VelocityYawRate v; ///< velocity
  Acceleration a;    ///< Acceleration
  Jerk j;            ///< Jerk
  Snap s;            ///< Snap
  double ga2;        ///< yaw-acceleration
                     /**
                     * @brief Add two states
                     * @param x State to add
                     * @return Sum of two states
                     */
  QuadFlatOutput operator+(const QuadFlatOutput &x) const {
    return QuadFlatOutput(this->p + x.p, this->v + x.v, this->a + x.a,
                          this->j + x.j, this->s + x.s, this->ga2 + x.ga2);
  }
  /**
  * @brief Subtract two states
  * @param x State to subtract
  * @return Difference of two states
  */
  QuadFlatOutput operator-(const QuadFlatOutput &x) const {
    return QuadFlatOutput(this->p - x.p, this->v - x.v, this->a - x.a,
                          this->j - x.j, this->s - x.s, this->ga2 - x.ga2);
  }
  // \todo add function to convert to pose and velocity
};
