#pragma once
#include "aerial_autonomy/controller_hardware_connectors/base_controller_hardware_connector.h"
#include "aerial_autonomy/types/empty_goal.h"
#include "aerial_autonomy/types/joystick_yaw.h"
#include "aerial_autonomy/types/roll_pitch_yaw_thrust.h"

#include <parsernode/parser.h>

/**
* @brief Maps Joystick goals to rpythrust commands to quadrotor
*/
class ManualRPYTControllerDroneConnector
    : public ControllerHardwareConnector<JoystickYaw, EmptyGoal,
                                         RollPitchYawThrust> {
public:
  /**
  * @brief Constructor
  *
  * Store drone hardware with hardware type as UAV.
  * Uses parsernode::Parser::cmdrpythrust function.
  *
  * @param drone_hardware Drone hardware used to send commands
  * @param controller RpyThrust controller
  */
  ManualRPYTControllerDroneConnector(
      parsernode::Parser &drone_hardware,
      Controller<JoystickYaw, EmptyGoal, RollPitchYawThrust> &controller)
      : ControllerHardwareConnector(controller, HardwareType::UAV),
        drone_hardware_(drone_hardware) {}

protected:
  /**
   * @brief Extracts joystick commands and current yaw from hardware
   *
   * @param sensor_data Joystick commands and current yaw
   *
   * @return true if succesfully extracted joystick data
   */
  virtual bool extractSensorData(JoystickYaw &sensor_data);

  /**
   * @brief  Send RPYT commands to hardware
   *
   * @param controls RPYT command to send to drone
   */
  virtual void sendHardwareCommands(RollPitchYawThrust controls);

private:
  /**
  * @brief Quad hardware to send commands
  */
  parsernode::Parser &drone_hardware_;
};
