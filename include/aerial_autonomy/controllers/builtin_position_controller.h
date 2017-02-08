#pragma once

#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/no_sensor.h"

class BuiltInPositionController : public Controller<NoSensor, PositionYaw, PositionYaw> {
  /**
   * @brief Run the control loop.  Simply returns the goal position.
   * @param sensor_data Data required for control loop. This 
   * controller required no data.
   * @return Position to send to hardware
   */
  virtual PositionYaw run(NoSensor sensor_data);
  /**
   * @brief Destructor
   */
  virtual ~BuiltInPositionController() {}
};
