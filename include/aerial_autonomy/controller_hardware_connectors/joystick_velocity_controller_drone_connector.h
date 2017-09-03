#pragma once
#include "aerial_autonomy/controller_hardware_connectors/base_controller_hardware_connector.h"
#include "aerial_autonomy/sensors/base_sensor.h"
#include "aerial_autonomy/types/empty_goal.h"
#include "aerial_autonomy/types/joystick.h"
#include "aerial_autonomy/types/roll_pitch_yaw_thrust.h"
#include "aerial_autonomy/types/velocity_yaw.h"

#include <parsernode/parser.h>

/**
* @brief Maps Joystick goals to velocity goals,
* used by controller to give rpythrust commands to quadrotor
*/
class JoystickVelocityControllerDroneConnector
    : public ControllerHardwareConnector<std::tuple<Joystick, VelocityYaw>,
                                         EmptyGoal, RollPitchYawThrust> {
public:
  /**
  * @brief Constructor
  *
  * Store drone hardware with hardware type as UAV.
  * Uses parsernode::Parser::cmdrpythrust function.
  *
  * @param drone_hardware Drone hardware used to send commands
  * @param controller Joystick velocity controller
  * @param velocity_sensor External velocity sensor
  */
  JoystickVelocityControllerDroneConnector(
      parsernode::Parser &drone_hardware,
      Controller<std::tuple<Joystick, VelocityYaw>, EmptyGoal,
                 RollPitchYawThrust> &controller,
      Sensor<VelocityYaw> &velocity_sensor);

protected:
  /**
   * @brief Extracts joystick commands from hardware
   * and velocity from sensor
   *
   * @param sensor_data Joystick commands and current velocity
   *
   * @return true if succesfully extracted joystick data
   */
  virtual bool
  extractSensorData(std::tuple<Joystick, VelocityYaw> &sensor_data);

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
  /**
  *  @brief sensor object to get sensor data
  */
  Sensor<VelocityYaw> &velocity_sensor_;
};
