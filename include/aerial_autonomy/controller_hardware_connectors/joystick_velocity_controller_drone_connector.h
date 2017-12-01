#pragma once
#include "aerial_autonomy/controller_hardware_connectors/base_controller_hardware_connector.h"
#include "aerial_autonomy/controllers/joystick_velocity_controller.h"
#include "aerial_autonomy/estimators/thrust_gain_estimator.h"
#include "aerial_autonomy/types/empty_goal.h"
#include "aerial_autonomy/types/joystick.h"
#include "aerial_autonomy/types/position.h"
#include "aerial_autonomy/types/roll_pitch_yawrate_thrust.h"
#include "aerial_autonomy/types/velocity_yaw_rate.h"

#include <parsernode/parser.h>

/**
* @brief Maps Joystick goals to velocity goals,
* used by controller to give rpythrust commands to quadrotor
*/
class JoystickVelocityControllerDroneConnector
    : public ControllerHardwareConnector<
          std::tuple<Joystick, VelocityYawRate, double>, EmptyGoal,
          RollPitchYawRateThrust> {
private:
  using BaseClass =
      ControllerHardwareConnector<std::tuple<Joystick, VelocityYawRate, double>,
                                  EmptyGoal, RollPitchYawRateThrust>;

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
      JoystickVelocityController &controller,
      ThrustGainEstimator &thrust_gain_estimator);

  /**
   * @brief set goal to controller and clear estimator buffer
   *
   * @param goal empty goal
   */
  void setGoal(EmptyGoal goal);

protected:
  /**
  * @brief extract sensor data from drone hardware
  *
  * @param sensor_data current joystick channels, velocity and yawrate, and
  * current yaw
  *
  * @return  false if there is an error in getting data from hardware
  */
  virtual bool
  extractSensorData(std::tuple<Joystick, VelocityYawRate, double> &sensor_data);
  /**
   * @brief  Send RPYT commands to hardware
   *
   * @param controls RPYT command to send to drone
   */
  virtual void sendHardwareCommands(RollPitchYawRateThrust controls);

private:
  /**
  * @brief Quad hardware to send commands
  */
  parsernode::Parser &drone_hardware_;
  ThrustGainEstimator &thrust_gain_estimator_;
  JoystickVelocityController &private_reference_controller_;
};
