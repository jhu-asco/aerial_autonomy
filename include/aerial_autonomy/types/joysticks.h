#pragma once
/**
* @brief 4channel Joystick data
*/
struct Joysticks {
  /**
  * @brief Constructor that implicitly instantiates channels to zero
  */
  Joysticks() : channel1(0), channel2(0), channel3(0), channel4(0) {}
  /**
  * @brief Constructor that explicitly takes in states
  *
  * @param channel1 First channel
  * @param channel2 Second channel
  * @param channel3 Third channel
  * @param channel4 Fourth channel
  */
  Joysticks(double channel1, double channel2, double channel3, double channel4)
      : channel1(channel1), channel2(channel2), channel3(channel3),
        channel4(channel4) {}
  double channel1; ///< First channel
  double channel2; ///< Second channel
  double channel3; ///< Third channel
  double channel4; ///< Fourth channel
};
