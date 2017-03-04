#pragma once
#include "aerial_autonomy/types/joysticks.h"

/**
* @brief Combined joystick and yaw data
*/
struct JoysticksYaw : public Joysticks {
  /**
  * @brief constructor that takes implicitly instantiates joystick and yaw data
  */
  JoysticksYaw() : Joysticks(), yaw(0) {}
  /**
  * @brief  Explicitly take in channel, yaw data
  *
  * @param channel1 First Channel
  * @param channel2 Second Channel
  * @param channel3 Third Channel
  * @param channel4 Fourth Channel
  * @param yaw Yaw data
  */
  JoysticksYaw(double channel1, double channel2, double channel3,
               double channel4, double yaw)
      : Joysticks(channel1, channel2, channel3, channel4), yaw(yaw) {}
  double yaw;///< Yaw data stored internally
};
