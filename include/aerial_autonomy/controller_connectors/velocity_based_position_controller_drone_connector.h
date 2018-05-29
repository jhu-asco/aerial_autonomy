#pragma once

#include "aerial_autonomy/controller_connectors/base_controller_connector.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/velocity_yaw_rate.h"

#include <parsernode/parser.h>

/**
 * @class VelocityBasedPositionControllerDroneConnector
 * @brief Manages communication between a drone plugin and a position controller
 * that outputs velocity commands.
 * \todo Gowtham Add connector tests
 */
class VelocityBasedPositionControllerDroneConnector
    : public ControllerConnector<PositionYaw, PositionYaw, VelocityYawRate> {
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
      : ControllerConnector(controller, ControllerGroup::UAV),
        drone_hardware_(drone_hardware) {}

protected:
  /**
   * @brief extracts position data from UAV to compute appropriate velocity
   * commands
   *
   * @param sensor_data Current position and yaw of UAV
   *
   * @return true if position and yaw can be extracted
   */
  virtual bool extractSensorData(PositionYaw &sensor_data);

  /**
   * @brief  Send velocity and yawrate commands to hardware
   *
   * @param controls velocity and yawrate commands to send to UAV
   */
  virtual void sendControllerCommands(VelocityYawRate controls);

private:
  /**
  * @brief Quad hardware to send commands
  */
  parsernode::Parser &drone_hardware_;
};
