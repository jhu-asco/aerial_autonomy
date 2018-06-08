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
  /**
  * @brief Control type
  */
  using ControlType = Eigen::VectorXd;
  /**
  * @brief State type
  */
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
  * @brief Initialize MPC controller
  */
  void initialize();

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

  /**
  * @brief Get the MPC trajectory
  *
  * @param xs vector of states
  * @param us vector of controls
  */
  void getTrajectory(std::vector<StateType> &xs,
                     std::vector<ControlType> &us) const;

  /**
  * @brief Get the reference MPC trajectory
  *
  * @param xds vector of desired states
  * @param uds vector of desired controls
  */
  void getDesiredTrajectory(std::vector<StateType> &xds,
                            std::vector<ControlType> &uds) const;

  /**
  * @brief get the pose of quadrotor as a tf transform
  *
  * @param data Quad data
  *
  * @return current pose of quadrotor
  */
  tf::Transform getPose(const parsernode::common::quaddata &data);

  /**
  * @brief Specify the time difference for differentiating position, joint
  * angles
  *
  * @param time_diff The simulated time difference between two measurements
  */
  void usePerfectTimeDiff(double time_diff = 0.02);

  /**
  * @brief Swap out the internal sensor with provided sensor
  *
  * @param sensor Pose sensor to use
  */
  void useSensor(SensorPtr<tf::StampedTransform> sensor);

protected:
  /**
  * @brief Set rpy command buffer to zeros and joint angles to current joint
  * angles
  */
  void clearCommandBuffers();

private:
  parsernode::Parser
      &drone_hardware_;     ///< Quadrotor parser for sending and receiving data
  ArmParser &arm_hardware_; ///< Parser for sending and receiving arm data
  SensorPtr<tf::StampedTransform> pose_sensor_; ///< Pose sensor for quad data
  ThrustGainEstimator &thrust_gain_estimator_;  ///< Thrust gain estimator
  std::vector<double> joint_angle_commands_;    ///< Commanded joint angles
  Eigen::VectorXd previous_measurements_;       ///< Previous measurements
  std::chrono::time_point<std::chrono::high_resolution_clock>
      previous_measurement_time_;          ///< Previous sensor measurement time
  bool previous_measurements_initialized_; ///< Flag to determin whether
                                           /// previous measurements have been
                                           /// initialized
  std::queue<Eigen::Vector3d> rpy_command_buffer_; ///< Rpy command buffer
  Eigen::Vector2d previous_joint_commands_;        ///< Previous joint commands
  Eigen::Vector2d filtered_joint_velocity_;        ///< Filtered joint velocity
  Eigen::Vector3d filtered_rpydot_;                ///< Filtered rpydot
  int delay_buffer_size_; ///< Size of rpy command buffer
  AbstractMPCController<StateType, ControlType>
      &private_controller_; ///< Private ref
  boost::mutex
      copy_mutex_; ///< Mutex for copying states and reference trajectories
  bool use_perfect_time_diff_; ///< Flag to use perfect time diff when
                               /// differentiating
  double perfect_time_diff_;   ///< The absolute time difference
  double exponential_gain_;    ///< Exponential gain
};
