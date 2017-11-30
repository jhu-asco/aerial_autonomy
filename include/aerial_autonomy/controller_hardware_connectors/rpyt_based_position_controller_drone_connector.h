#pragma once

#include "aerial_autonomy/controller_hardware_connectors/base_controller_hardware_connector.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/roll_pitch_yawrate_thrust.h"
#include "aerial_autonomy/types/velocity_yaw_rate.h"

#include <parsernode/parser.h>
/**
 * @brief Manages communication between a UAV plugin and a position controller
 * that outputs a rpyt command
 */
class RPYTBasedPositionControllerDroneConnector
    : public ControllerHardwareConnector<
          std::tuple<VelocityYawRate, PositionYaw>, PositionYaw,
          RollPitchYawRateThrust> {
public:
  /**
  * @brief Constructor
  * @param drone_hardware Plugin used to send commands to UAV
  * @param controller Position controller that gives rpyt commands
  */
  RPYTBasedPositionControllerDroneConnector(
      parsernode::Parser &drone_hardware,
      Controller<std::tuple<VelocityYawRate, PositionYaw>, PositionYaw,
                 RollPitchYawRateThrust> &controller)
      : ControllerHardwareConnector(controller, HardwareType::UAV),
        drone_hardware_(drone_hardware) {}

protected:
  /**
   * @brief extracts position and velocity data from UAV to compute appropriate
   * rpyt
   * commands
   *
   * @param sensor_data Current position and velocity of UAV
   *
   * @return true if sensor data can be extracted
   */
  virtual bool
  extractSensorData(std::tuple<VelocityYawRate, PositionYaw> &sensor_data);

  /**
   * @brief  Send rpyt commands to hardware
   *
   * @param controls rpyt commands to send to UAV
   */
  virtual void sendHardwareCommands(RollPitchYawRateThrust controls);

private:
  /**
  * @brief Quad hardware to send commands
  */
  parsernode::Parser &drone_hardware_;
};
