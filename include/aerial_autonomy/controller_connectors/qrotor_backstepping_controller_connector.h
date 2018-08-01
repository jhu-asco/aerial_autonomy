#pragma once

#include "aerial_autonomy/controller_connectors/base_controller_connector.h"
#include "aerial_autonomy/controllers/rpyt_based_position_controller.h"
#include "aerial_autonomy/estimators/thrust_gain_estimator.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/roll_pitch_yawrate_thrust.h"
#include "aerial_autonomy/types/velocity_yaw_rate.h"

#include <chrono>
#include <parsernode/parser.h>

/**
 * @brief A controller connector for trajectory-tracking backstepping controller
 * for a quadrotor.
 * Based on <Kobilarov, "Trajectory tracking of a class of underactuated systems
 * with
 * external disturbances", American Control Conference, 2013>
 */
class QrotorBacksteppingControllerConnector
    : public ControllerConnector<
          std::pair<double, QrotorBacksteppingState>, // SensorDataType
          std::shared_ptr<ReferenceTrajectory<ParticleState, Snap>>, // GoalType
          QrotorBacksteppingControl> { // ControlType
public:
  /**
  * @brief Constructor
  * @param drone_hardware Plugin used to send commands to UAV
  * @param controller Position controller that gives rpyt commands
  */
  QrotorBacksteppingControllerConnector(
      parsernode::Parser &drone_hardware,
      QrotorBacksteppingController &controller,
      ThrustGainEstimator &thrust_gain_estimator)
      : ControllerConnector(controller, ControllerGroup::UAV),
        drone_hardware_(drone_hardware),
        thrust_gain_estimator_(thrust_gain_estimator),
        private_reference_controller_(controller),
        t_0_(std::chrono::high_resolution_clock::now()) {}

  /**
   * @brief set goal to controller and clear estimator buffer
   *
   * @param goal empty goal
   */
  void setGoal(std::shared_ptr<ReferenceTrajectory<ParticleState, Snap>> goal);

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
  extractSensorData(std::pair<double, QrotorBacksteppingState> &sensor_data);

  /**
   * @brief  Send rpyt commands to hardware
   *
   * @param controls rpyt commands to send to UAV
   */
  virtual void sendControllerCommands(QrotorBacksteppingControl control);

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
  /**
   * @brief Internal reference to controller that is connected by this class
   */
  QrotorBacksteppingController &private_reference_controller_;

  std::chrono::time_point<std::chrono::high_resolution_clock> t_0_;
