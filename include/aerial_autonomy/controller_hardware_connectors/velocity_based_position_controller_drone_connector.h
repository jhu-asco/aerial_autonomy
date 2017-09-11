#pragma once

#include "aerial_autonomy/controller_hardware_connectors/base_controller_hardware_connector.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/velocity_yaw_rate.h"

#include <parsernode/parser.h>

/**
 * @class VelocityBasedPositionControllerDroneConnector
 * @brief Manages communication between a drone plugin and a position controller
 * that outputs
 * position commands.
 */
class VelocityBasedPositionControllerDroneConnector
    : public ControllerHardwareConnector<PositionYaw, PositionYaw,
                                         VelocityYawRate> {
public:
  /**
  * @brief Constructor
  *
  * Store drone hardware with hardware type as UAV.
  * Uses parsernode::Parser::cmdwaypoint function.
  *
  * @param drone_hardware Drone hardware used to send commands
  * @param controller Position controller that achieves a desired position, yaw
  */
  VelocityBasedPositionControllerDroneConnector(
      parsernode::Parser &drone_hardware,
      Controller<PositionYaw, PositionYaw, VelocityYawRate> &controller)
      : ControllerHardwareConnector(controller, HardwareType::UAV),
        drone_hardware_(drone_hardware) {}

protected:
  /**
   * @brief  does not extract any data since nothing is needed
   *
   * @param sensor_data Current position and yaw of UAV
   *
   * @return true if position and yaw can be extracted
   */
  virtual bool extractSensorData(PositionYaw &sensor_data);

  /**
   * @brief  Send position commands to hardware
   *
   * @param controls position command to send to UAV
   */
  virtual void sendHardwareCommands(VelocityYawRate controls);

private:
  /**
  * @brief Quad hardware to send commands
  */
  parsernode::Parser &drone_hardware_;
};
