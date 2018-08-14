#pragma once

#include "aerial_autonomy/controller_connectors/base_controller_connector.h"
#include "aerial_autonomy/controllers/rpyt_based_position_controller.h"
#include "aerial_autonomy/estimators/thrust_gain_estimator.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/roll_pitch_yawrate_thrust.h"
#include "aerial_autonomy/types/velocity_yaw_rate.h"
#include "aerial_autonomy/sensors/base_sensor.h"

#include <parsernode/parser.h>
/**
 * @brief Manages communication between a UAV plugin and a position controller
 * that outputs a rpyt command
 */
class RPYTBasedPositionControllerDroneConnector
    : public ControllerConnector<std::tuple<VelocityYawRate, PositionYaw>,
                                 PositionYaw, RollPitchYawRateThrust> {
public:
  /**
  * @brief Constructor
  * @param drone_hardware Plugin used to send commands to UAV
  * @param controller Position controller that gives rpyt commands
  */
  RPYTBasedPositionControllerDroneConnector(
      parsernode::Parser &drone_hardware,
      RPYTBasedPositionController &controller,
      ThrustGainEstimator &thrust_gain_estimator,
      Sensor<std::tuple<VelocityYawRate, PositionYaw>> *sensor = nullptr)
      : ControllerConnector(controller, ControllerGroup::UAV),
        drone_hardware_(drone_hardware),
        thrust_gain_estimator_(thrust_gain_estimator),
        private_reference_controller_(controller),
        sensor_(sensor) {}
  /**
   * @brief set goal to controller and clear estimator buffer
   *
   * @param goal empty goal
   */
  void setGoal(PositionYaw goal);

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
  virtual void sendControllerCommands(RollPitchYawRateThrust controls);

private:
  /**
   * @brief Base class typedef to simplify code
   */
  using BaseClass =
      ControllerConnector<std::tuple<VelocityYawRate, PositionYaw>, PositionYaw,
                          RollPitchYawRateThrust>;
  /**
  * @brief Quad hardware to send commands
  */
  parsernode::Parser &drone_hardware_;
  /**
   * @brief Estimator for finding the gain between joystick thrust command and
   * the acceleration in body z direction
   */
  ThrustGainEstimator &thrust_gain_estimator_;
  /**
   * @brief Internal reference to controller that is connected by this class
   */
  RPYTBasedPositionController &private_reference_controller_;

  /**
   * @brief Outside Odometry Sensor (default to null, change to use different sensors)
   */
  Sensor<std::tuple<VelocityYawRate, PositionYaw>> *sensor_;

};
