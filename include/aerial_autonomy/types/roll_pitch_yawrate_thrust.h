#pragma once
/**
* @brief Roll, pitch, yaw rate, and thrust message
*/
struct RollPitchYawRateThrust {
  /**
  * @brief Explicit instantiation
  *
  * @param r roll in rad
  * @param p pitch in rad
  * @param y yaw rate in rad/s
  * @param t Thrust
  */
  RollPitchYawRateThrust(double r = 0, double p = 0, double y = 0, double t = 0)
      : r(r), p(p), y(y), t(t) {}
  double r; ///< roll
  double p; ///< pitch
  double y; ///< yaw rate
  double t; ///< thrust
};
