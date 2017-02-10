#pragma once
#include "aerial_autonomy/controller_hardware_connectors/base_controller_hardware_connector.h"
#include "aerial_autonomy/types/empty_goal.h"
#include "aerial_autonomy/types/joysticks_yaw.h"
#include "aerial_autonomy/types/roll_pitch_yaw_thrust.h"

#include <parsernode/parser.h>

class ManualRPYTControllerDroneConnector
    : public ControllerHardwareConnector<JoysticksYaw, EmptyGoal,
                                         RollPitchYawThrust> {
public:
  ManualRPYTControllerDroneConnector(
      parsernode::Parser &drone_hardware,
      Controller<JoysticksYaw, EmptyGoal, RollPitchYawThrust> &controller)
      : ControllerHardwareConnector(controller),
        drone_hardware_(drone_hardware) {}

protected:
  /**
   * @brief  does not extract any data since nothing is needed
   *
   * @return empty data structure
   */
  virtual JoysticksYaw extractSensorData();

  /**
   * @brief  Send velocity commands to hardware
   *
   * @param controls velocity command to send to quadrotor
   */
  virtual void sendHardwareCommands(RollPitchYawThrust controls);

private:
  parsernode::Parser &drone_hardware_;
};
