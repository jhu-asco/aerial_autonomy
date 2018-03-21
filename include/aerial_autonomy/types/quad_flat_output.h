#pragma once
#include "aerial_autonomy/types/acceleration.h"
#include "aerial_autonomy/types/jerk.h"
#include "aerial_autonomy/types/position_yaw.h"
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
                 Acceleration acceleration, Jerk jerk)
      : p(position_yaw), v(velocity_yaw_rate), a(acceleration), j(jerk) {}

  PositionYaw p;
  VelocityYawRate v;
  Acceleration a;
  Jerk j;
};
