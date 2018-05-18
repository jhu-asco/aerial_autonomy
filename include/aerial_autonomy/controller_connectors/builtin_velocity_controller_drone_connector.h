#pragma once

#include "aerial_autonomy/controller_connectors/base_controller_connector.h"
#include "aerial_autonomy/types/velocity_yaw.h"

#include <parsernode/parser.h>

/**
 * @class BuiltInVelocityControllerDroneConnector
 * @brief Manages communication between a drone plugin and a velocity controller
 * that outputs velocity commands.
 */
class BuiltInVelocityControllerDroneConnector
    : public ControllerConnector<VelocityYaw, VelocityYaw, VelocityYaw> {
public:
  /**
  * @brief Constructor
  *
  * Store drone hardware with hardware type as UAV.
  * Uses parsernode::Parser::cmdvelguided function.
  *
  * @param drone_hardware Drone hardware used to send commands
  * @param controller Velocity controller that achieves a desired velocity, yaw
  */
  BuiltInVelocityControllerDroneConnector(
      parsernode::Parser &drone_hardware,
      Controller<VelocityYaw, VelocityYaw, VelocityYaw> &controller)
      : ControllerConnector(controller, ControllerGroup::UAV),
        drone_hardware_(drone_hardware) {}

protected:
  /**
   * @brief  does not extract any data since nothing is needed
   *
   * @param sensor_data current velocity and yaw of UAV
   *
   * @return true if velocity and yaw can be found
   */
  virtual bool extractSensorData(VelocityYaw &sensor_data);

  /**
   * @brief  Send velocity commands to hardware
   *
   * @param controls velocity command to send to UAV
   */
  virtual void sendControllerCommands(VelocityYaw controls);

private:
  /**
  * @brief Quad hardware to send commands
  */
  parsernode::Parser &drone_hardware_;
};
