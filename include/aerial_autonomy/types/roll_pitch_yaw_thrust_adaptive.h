#pragma once
#include "aerial_autonomy/types/roll_pitch_yaw_thrust.h"
/**
* @brief Roll, pitch, yaw, thrust and adaptive parameter estimate message
*/

struct RollPitchYawThrustAdaptive : RollPitchYawThrust {
  /**
  * @brief Explicit instantiation
  *
  * @param r roll in rad
  * @param p pitch in rad
  * @param y yaw in rad
  * @param t Thrust
  * @param dm derivative of mass perimeter
  */
  double dm; ///< dm
};
