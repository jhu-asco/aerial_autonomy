#pragma once

#include "aerial_autonomy/controller_hardware_connectors/base_controller_hardware_connector.h"
#include "aerial_autonomy/types/empty_sensor.h"
#include "aerial_autonomy/types/position_yaw.h"

#include <parsernode/parser.h>

/**
 * @class PositionControllerDroneConnector
 * @brief Manages communication between a drone plugin and a position controller
 * that outputs
 * position commands.
 */
class PositionControllerDroneConnector
    : public ControllerHardwareConnector<EmptySensor, PositionYaw,
                                         PositionYaw> {
public:
  PositionControllerDroneConnector(
      parsernode::Parser &drone_hardware,
      Controller<EmptySensor, PositionYaw, PositionYaw> &controller)
      : ControllerHardwareConnector(controller, HardwareType::Quadrotor),
        drone_hardware_(drone_hardware) {}

protected:
  /**
   * @brief  does not extract any data since nothing is needed
   *
   * @return empty data structure
   */
  virtual EmptySensor extractSensorData();

  /**
   * @brief  Send position commands to hardware
   *
   * @param controls position command to send to quadrotor
   */
  virtual void sendHardwareCommands(PositionYaw controls);

private:
  parsernode::Parser &drone_hardware_;
};
