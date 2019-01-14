#pragma once
#include "aerial_autonomy/types/roll_pitch_yaw_thrust.h"
/**
* @brief Roll, pitch, yaw, thrust and adaptive parameter estimate message
*/

struct RollPitchYawThrustAdaptive {
  /**
  * @brief Explicit instantiation
  *
  * @param r roll in rad
  * @param p pitch in rad
  * @param y yaw in rad
  * @param t Thrust
  * @param dm derivative of mass perimeter
  */
  RollPitchYawThrustAdaptive(double r = 0, double p = 0, double y = 0,
                             double t = 0, double dm = 0)
      : rpyt(r, p, y, t), dm(dm) {}
  RollPitchYawThrust rpyt;
  double dm; ///< dm
};
