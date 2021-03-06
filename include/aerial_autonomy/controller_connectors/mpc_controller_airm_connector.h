#pragma once
#include "aerial_autonomy/controller_connectors/base_mpc_controller_quad_connector.h"
#include <Eigen/Dense>
#include <arm_parsers/arm_parser.h>
#include <tf/tf.h>

/**
* @brief Controller connector for generic MPC Controller for Quadrotor and arm
* system
*/
class MPCControllerAirmConnector : public BaseMPCControllerQuadConnector {
public:
  /**
  * @brief Constructor
  *
  * @param drone_hardware Quadrotor parser that facilitates sending commands to
  * the robot
  * @param arm_hardware Arm parser for sending commands to arm hardware
  * @param controller The MPC Controller to use
  * @param constraint_generator The constraint generator to use
  * @param state_estimator state estimator that finds the state of robot
  */
  MPCControllerAirmConnector(
      parsernode::Parser &drone_hardware, ArmParser &arm_hardware,
      AbstractMPCController<StateType, ControlType> &controller,
      ThrustGainEstimator &thrust_gain_estimator, int delay_buffer_size = 1,
      MPCConnectorConfig config = MPCConnectorConfig(),
      SensorPtr<std::pair<tf::StampedTransform, tf::Vector3>> odom_sensor =
          nullptr,
      AbstractConstraintGeneratorPtr constraint_generator = nullptr);

  /**
  * @brief Initialize MPC controller
  */
  virtual void initialize();

  /**
  * @brief send commands to Quadrotor and arm
  *
  * @param control The commanded roll, pitch, yaw and thrust to send to
  * quadrotor and joint angles to send to arm
  */
  void sendControllerCommands(ControlType control);

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

protected:
  /**
  * @brief Set rpy command buffer to zeros and joint angles to current joint
  * angles
  */
  void clearJointCommandBuffers();

  /**
   * @brief Get time difference
   *
   * @return time difference
   */
  double getTimeDiff();

private:
  ArmParser &arm_hardware_; ///< Parser for sending and receiving arm data
  std::vector<double> joint_angle_commands_; ///< Commanded joint angles
  Eigen::Vector2d previous_joint_commands_;  ///< Previous joint commands
  Eigen::Vector2d previous_joint_angles_;    ///< Previous joint angles
  ExponentialFilter<Eigen::Vector2d>
      joint_velocity_filter_; ///< Filter joint velocities
  bool
      previous_joint_measurements_initialized_; ///< Previous joint measurements
  static constexpr int state_size_ = 21;
  std::chrono::time_point<std::chrono::high_resolution_clock>
      previous_measurement_time_; ///< For finding time diff
};
