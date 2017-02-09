#pragma once

#include "aerial_autonomy/controller_hardware_connectors/base_controller_hardware_connector.h"
#include "aerial_autonomy/types/empty_sensor.h"
#include "aerial_autonomy/types/velocity_yaw.h"

#include <parsernode/parser.h>

/**
 * @class BuiltInVelocityControllerDroneConnector
 * @brief Manages communication between a drone plugin and a velocity controller
 * that outputs velocity commands.
 */
class BuiltInVelocityControllerDroneConnector
    : public ControllerHardwareConnector<EmptySensor, VelocityYaw,
                                         VelocityYaw> {
public:
  BuiltInVelocityControllerDroneConnector(
      parsernode::Parser &drone_hardware,
      Controller<EmptySensor, VelocityYaw, VelocityYaw> &controller)
      : ControllerHardwareConnector(controller),
        drone_hardware_(drone_hardware) {}

protected:
  /**
   * @brief  does not extract any data since nothing is needed
   *
   * @return empty data structure
   */
  virtual EmptySensor extractSensorData();

  /**
   * @brief  Send velocity commands to hardware
   *
   * @param controls velocity command to send to quadrotor
   */
  virtual void sendHardwareCommands(VelocityYaw controls);

private:
  parsernode::Parser &drone_hardware_;
};
