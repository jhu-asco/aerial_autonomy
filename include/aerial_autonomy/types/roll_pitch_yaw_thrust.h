#pragma once
/**
* @brief Roll, pitch, yaw, and thrust message
*/
struct RollPitchYawThrust {
  /**
  * @brief Implicit instantiation to zero
  */
  RollPitchYawThrust() : r(0), p(0), y(0), t(0) {}
  /**
  * @brief Explicit instantiation
  *
  * @param r roll in rad
  * @param p pitch in rad
  * @param y yaw in rad
  * @param t Thrust
  */
  RollPitchYawThrust(double r, double p, double y, double t)
      : r(r), p(p), y(y), t(t) {}
  double r; ///< roll
  double p; ///< pitch
  double y; ///< yaw
  double t; ///< thrust
};
