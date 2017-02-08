#pragma once

#include "aerial_autonomy/controller_hardware_connectors/base_control_hardware_connector.h"
#include "aerial_autonomy/types/no_sensor.h"
#include "aerial_autonomy/types/position_yaw.h"

#include <parsernode/parser.h>

class PositionControllerDroneConnector : 
  public ControllerHardwareConnector<NoSensor, PositionYaw, PositionYaw> {
public:
  PositionControllerDroneConnector(parsernode::Parser& drone_hardware, Controller<NoSensor, PositionYaw, PositionYaw> &controller) :
    ControllerHardwareConnector(controller), drone_hardware_(drone_hardware) {}

protected:
  /**
   * @brief  does not extract any data since nothing is needed
   *
   * @return empty data structure
   */
  virtual NoSensor extractSensorData();

  /**
   * @brief  Send position commands to hardware
   *
   * @param controls position command to send to quadrotor
   */
  virtual void sendHardwareCommands(PositionYaw controls);

private:
  parsernode::Parser& drone_hardware_;
};
