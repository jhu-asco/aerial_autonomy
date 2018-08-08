#pragma once
#include "aerial_autonomy/controller_connectors/quad_airm_common_mpc_connector.h"
#include <Eigen/Dense>
#include <tf/tf.h>

/**
* @brief Controller connector for generic MPC Controller for Quadrotor
* system
*/
class MPCControllerQuadConnector : public QuadAirmMPCCommonConnector {
public:
  /**
  * @brief Constructor
  *
  * @param drone_hardware Quadrotor parser that facilitates sending commands to
  * the robot
  * @param controller The MPC Controller to use
  * @param constraint_generator The constraint generator to use
  * @param state_estimator state estimator that finds the state of robot
  */
  MPCControllerQuadConnector(
      parsernode::Parser &drone_hardware,
      AbstractMPCController<StateType, ControlType> &controller,
      ThrustGainEstimator &thrust_gain_estimator, int delay_buffer_size = 1,
      SensorPtr<tf::StampedTransform> pose_sensor = nullptr,
      AbstractConstraintGeneratorPtr constraint_generator = nullptr);

  /**
  * @brief Initialize MPC controller
  */
  void initialize();

  /**
  * @brief Estimate the current state and static MPC parameters
  *
  * @param current_state Current system state
  * @param params Current MPC parameters
  *
  * @return true if estimation is successful
  */
  bool estimateStateAndParameters(Eigen::VectorXd &current_state,
                                  Eigen::VectorXd &params);

private:
  Eigen::VectorXd previous_measurements_; ///< Previous measurements
  static constexpr int state_size_ = 15;  ///< State size
};
