#pragma once
#include "aerial_autonomy/controller_connectors/base_controller_connector.h"
#include "aerial_autonomy/estimators/thrust_gain_estimator.h"
#include "aerial_autonomy/types/empty_goal.h"
#include "aerial_autonomy/types/joystick.h"
#include "aerial_autonomy/types/roll_pitch_yawrate_thrust.h"

#include <parsernode/parser.h>

/**
* @brief Maps Joystick goals to rpythrust commands to quadrotor
*/
class ManualRPYTControllerDroneConnector
    : public ControllerConnector<Joystick, EmptyGoal, RollPitchYawRateThrust> {
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
      Controller<Joystick, EmptyGoal, RollPitchYawRateThrust> &controller,
      ThrustGainEstimator &thrust_gain_estimator)
      : ControllerConnector(controller, ControllerGroup::UAV),
        drone_hardware_(drone_hardware),
        thrust_gain_estimator_(thrust_gain_estimator) {}

protected:
  /**
   * @brief Extracts joystick commands and current yaw from hardware
   *
   * @param sensor_data Joystick commands and current yaw
   *
   * @return true if succesfully extracted joystick data
   */
  virtual bool extractSensorData(Joystick &sensor_data);

  /**
   * @brief  Send RPYT commands to hardware
   *
   * @param controls RPYT command to send to drone
   */
  virtual void sendControllerCommands(RollPitchYawRateThrust controls);

private:
  /**
  * @brief Quad hardware to send commands
  */
  parsernode::Parser &drone_hardware_;
  /**
   * @brief Estimator for finding the gain between joystick thrust command and
   * the acceleration in body z direction
   */
  ThrustGainEstimator &thrust_gain_estimator_;
};
