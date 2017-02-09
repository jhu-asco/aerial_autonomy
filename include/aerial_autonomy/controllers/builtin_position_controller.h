#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/empty_sensor.h"

/**
 * @brief A position controller that simply outputs the set goal position
 */
class BuiltInPositionController
    : public Controller<EmptySensor, PositionYaw, PositionYaw> {
public:
  /**
   * @brief Destructor
   */
  virtual ~BuiltInPositionController() {}

protected:
  /**
   * @brief Run the control loop.  Simply returns the goal position.
   * @param sensor_data Empty sensor data struct since no sensing is required.
   * @param goal Position set-point
   * @return Position to send to hardware
   */
  virtual PositionYaw runImplementation(EmptySensor sensor_data,
                                        PositionYaw goal);
};
