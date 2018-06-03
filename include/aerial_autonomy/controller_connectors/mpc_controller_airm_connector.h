#pragma once
#include "aerial_autonomy/controller_connectors/mpc_controller_connector.h"
#include "aerial_autonomy/estimators/thrust_gain_estimator.h"
#include "aerial_autonomy/sensors/base_sensor.h"
#include "aerial_autonomy/types/arm_state.h"
#include "aerial_autonomy/types/quad_state.h"
#include "aerial_autonomy/types/roll_pitch_yaw_thrust.h"
#include <Eigen/Dense>
#include <arm_parsers/arm_parser.h>
#include <chrono>
#include <parsernode/parser.h>
#include <queue>
#include <tf/tf.h>

/**
* @brief Controller connector for generic MPC Controller for Quadrotor and arm
* system
*/
class MPCControllerAirmConnector
    : public MPCControllerConnector<Eigen::VectorXd, Eigen::VectorXd> {
public:
  using ControlType = Eigen::VectorXd;
  using StateType = Eigen::VectorXd;
  /**
  * @brief Constructor
  *
  * @param drone_hardware Quadrotor parser that facilitates sending commands to
  * the robot
  * @param controller The MPC Controller to use
  * @param constraint_generator The constraint generator to use
  * @param state_estimator state estimator that finds the state of robot
  */
  MPCControllerAirmConnector(
      parsernode::Parser &drone_hardware, ArmParser &arm_hardware,
      AbstractMPCController<StateType, ControlType> &controller,
      ThrustGainEstimator &thrust_gain_estimator, int delay_buffer_size = 1,
      SensorPtr<tf::StampedTransform> pose_sensor = nullptr,
      AbstractConstraintGeneratorPtr constraint_generator = nullptr);

  /**
  * @brief Set the goal for controller
  *
  * @param goal the designated goal
  */
  void setGoal(ReferenceTrajectoryPtr<StateType, ControlType> goal);

  /**
  * @brief send commands to Quadrotor and arm
  *
  * @param control The commanded roll, pitch, yaw and thrust to send to
  * quadrotor and joint angles to send to arm
  */
  void sendControllerCommands(ControlType control);

  bool estimateStateAndParameters(Eigen::VectorXd &current_state,
                                  Eigen::VectorXd &params);

  void getTrajectory(std::vector<StateType> &xs,
                     std::vector<ControlType> &us) const;

  void getDesiredTrajectory(std::vector<StateType> &xds,
                            std::vector<ControlType> &uds) const;

  tf::Transform getPose(const parsernode::common::quaddata &data);

protected:
  void clearCommandBuffers();

private:
  parsernode::Parser
      &drone_hardware_;     ///< Quadrotor parser for sending and receiving data
  ArmParser &arm_hardware_; ///< Parser for sending and receiving arm data
  SensorPtr<tf::StampedTransform> pose_sensor_; ///< Pose sensor for quad data
  ThrustGainEstimator &thrust_gain_estimator_;
  std::vector<double> joint_angle_commands_; ///< Commanded joint angles
  Eigen::VectorXd previous_measurements_;    ///< Previous measurements
  std::chrono::time_point<std::chrono::high_resolution_clock>
      previous_measurement_time_;          ///< Previous sensor measurement time
  bool previous_measurements_initialized_; ///< Flag to determin whether
                                           /// previous measurements have been
                                           /// initialized
  std::queue<Eigen::Vector3d> rpy_command_buffer_; ///< Rpy command buffer
  Eigen::Vector2d previous_joint_commands_;        ///< Previous joint commands
  int delay_buffer_size_; ///< Size of rpy command buffer
  AbstractMPCController<StateType, ControlType>
      &private_controller_; ///< Private ref
  boost::mutex
      copy_mutex_; ///< Mutex for copying states and reference trajectories
};
