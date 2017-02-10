#pragma once
#include "aerial_autonomy/types/joysticks.h"

struct JoysticksYaw : public Joysticks {
  JoysticksYaw() : Joysticks(), yaw(0) {}
  JoysticksYaw(double channel1, double channel2, double channel3,
               double channel4, double yaw)
      : Joysticks(channel1, channel2, channel3, channel4), yaw(yaw) {}
  double yaw;
};
