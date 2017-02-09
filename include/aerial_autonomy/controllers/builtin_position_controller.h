#pragma once

#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/no_sensor.h"

/**
 * @brief A position controller that simply outputs the set goal position
 */
class BuiltInPositionController : public Controller<NoSensor, PositionYaw, PositionYaw> {
public:
  /**
   * @brief Run the control loop.  Simply returns the goal position.
   * @param sensor_data Empty sensor data struct since no sensing is required.
   * @param goal Position set-point
   * @return Position to send to hardware
   */
  virtual PositionYaw runImpl(NoSensor sensor_data, PositionYaw goal);
  /**
   * @brief Destructor
   */
  virtual ~BuiltInPositionController() {}
};
