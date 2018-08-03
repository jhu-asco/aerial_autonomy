#pragma once

#include "aerial_autonomy/controller_connectors/base_controller_connector.h"
#include "aerial_autonomy/controllers/qrotor_backstepping_controller.h"
#include "aerial_autonomy/estimators/thrust_gain_estimator.h"
#include "aerial_autonomy/types/roll_pitch_yawrate_thrust.h"
#include "qrotor_backstepping_controller_config.pb.h"

#include <Eigen/Cholesky>
#include <Eigen/Dense>
#include <Eigen/LU>
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
      ThrustGainEstimator &thrust_gain_estimator,
      QrotorBacksteppingControllerConfig config,
      std::chrono::duration<double> controller_timer_duration)
      : ControllerConnector(controller, ControllerGroup::UAV),
        drone_hardware_(drone_hardware),
        thrust_gain_estimator_(thrust_gain_estimator),
        private_reference_controller_(controller),
        t_0_(std::chrono::high_resolution_clock::now()), config_(config),
        m_(config_.mass()), g_(config_.acc_gravity()),
        dt_(controller_timer_duration.count()) {
    J_ << config_.jxx(), config_.jxy(), config_.jxz(), config_.jyx(),
        config_.jyy(), config_.jyz(), config_.jzx(), config_.jzy(),
        config_.jzz();
  }

  /**
  * @brief set goal to controller and clear estimator buffer
  *
  * @param goal empty goal
  */
  void setGoal(std::shared_ptr<ReferenceTrajectory<ParticleState, Snap>> goal);

  void setThrust(double thrust) { thrust_ = thrust; }
  double getThrust() { return thrust_; }
  void setThrustDot(double thrust_dot) { thrust_dot_ = thrust_dot; }
  double getThrustDot() { return thrust_dot_; }
  // QrotorBacksteppingControllerConfig getConfig() { return config_; }

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

  /**
  * @brief  Mapping from body angular velocity to local rpydot
  *
  * @param omega body angular velocity
  * @param rpy current rpy from sensor data
  *
  * @return eigen vector3d, rpydot
  */
  Eigen::Vector3d omegaToRpyDot(const Eigen::Vector3d &omega,
                                const Eigen::Vector3d &rpy);

private:
  /**
  * @brief Base class typedef to simplify code
  */
  using BaseClass = ControllerConnector<
      std::pair<double, QrotorBacksteppingState>,
      std::shared_ptr<ReferenceTrajectory<ParticleState, Snap>>,
      QrotorBacksteppingControl>;
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
  /**
  * @brief time at start
  */
  std::chrono::time_point<std::chrono::high_resolution_clock> t_0_;
  /**
  * @brief Controller config
  */
  QrotorBacksteppingControllerConfig config_;
  /**
  * @brief Variable to store current thrust
  */
  double thrust_;
  /**
  * @brief Variable to store current thrust_dot
  */
  double thrust_dot_;
  /**
  * @brief Mass
  */
  double m_;
  /**
  * @brief Gravity
  */
  double g_;
  /**
  * @brief Moment of inertia matrix
  */
  Eigen::Matrix3d J_;
  /**
  * @brief Time duration for numerical integration
  */
  double dt_;

  parsernode::common::quaddata data_;

  QrotorBacksteppingState current_state_;
};
