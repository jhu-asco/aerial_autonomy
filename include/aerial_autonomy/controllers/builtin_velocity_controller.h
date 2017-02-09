#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/empty_sensor.h"
#include "aerial_autonomy/types/velocity_yaw.h"

/**
 * @brief A velocity controller that simply outputs the set goal velocity
 */
class BuiltInVelocityController
    : public Controller<EmptySensor, VelocityYaw, VelocityYaw> {
public:
  /**
   * @brief Destructor
   */
  virtual ~BuiltInVelocityController() {}

protected:
  /**
   * @brief Run the control loop.  Simply returns the goal velocity.
   * @param sensor_data Empty sensor data struct since no sensing is required.
   * @param goal Velocity set-point
   * @return Velocity to send to hardware
   */
  virtual VelocityYaw runImplementation(EmptySensor sensor_data,
                                        VelocityYaw goal);
};
