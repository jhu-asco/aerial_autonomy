#pragma once
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
      : r(r), p(p), y(y), t(t), dm(dm) {}
  double r;  ///< roll
  double p;  ///< pitch
  double y;  ///< yaw
  double t;  ///< thrust
  double dm; ///< dm
};
