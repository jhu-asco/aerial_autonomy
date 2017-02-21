#pragma once
struct Joysticks {
  Joysticks() : channel1(0), channel2(0), channel3(0), channel4(0) {}
  Joysticks(double channel1, double channel2, double channel3, double channel4)
      : channel1(channel1), channel2(channel2), channel3(channel3),
        channel4(channel4) {}
  double channel1;
  double channel2;
  double channel3;
  double channel4;
};
