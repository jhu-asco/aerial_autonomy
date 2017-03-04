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
      Controller<EmptySensor, VelocityYaw, VelocityYaw> &controller)
      : ControllerHardwareConnector(controller, HardwareType::UAV),
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
   * @param controls velocity command to send to UAV
   */
  virtual void sendHardwareCommands(VelocityYaw controls);

private:
  /**
  * @brief Quad hardware to send commands
  */
  parsernode::Parser &drone_hardware_;
};
