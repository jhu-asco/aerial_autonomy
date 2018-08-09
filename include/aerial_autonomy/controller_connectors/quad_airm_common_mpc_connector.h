#pragma once
#include "aerial_autonomy/controller_connectors/mpc_controller_connector.h"
#include "aerial_autonomy/estimators/thrust_gain_estimator.h"
#include "aerial_autonomy/sensors/base_sensor.h"
#include <Eigen/Dense>
#include <chrono>
#include <parsernode/parser.h>
#include <queue>
#include <tf/tf.h>

/**
* @brief Controller connector common code between airm and quadrotor
* system
*/
class QuadAirmMPCCommonConnector
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
  QuadAirmMPCCommonConnector(
      parsernode::Parser &drone_hardware,
      AbstractMPCController<StateType, ControlType> &controller,
      ThrustGainEstimator &thrust_gain_estimator, int delay_buffer_size = 1,
      SensorPtr<tf::StampedTransform> pose_sensor = nullptr,
      AbstractConstraintGeneratorPtr constraint_generator = nullptr);

  /**
   * @brief ~QuadAirmMPCCommonConnector Destructor
   */
  virtual ~QuadAirmMPCCommonConnector() {}

  /**
  * @brief send commands to Quadrotor
  *
  * @param control The commanded roll, pitch, yaw and thrust to send to
  * quadrotor
  */
  virtual void sendControllerCommands(ControlType control);

  /**
  * @brief Specify the time difference for finite differentiation
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
  * @brief Set rpy command buffer to zeros
  */
  void clearCommandBuffers();

protected:
  parsernode::Parser
      &drone_hardware_; ///< Quadrotor parser for sending and receiving data
  SensorPtr<tf::StampedTransform> pose_sensor_; ///< Pose sensor for quad data
  ThrustGainEstimator &thrust_gain_estimator_;  ///< Thrust gain estimator
  std::chrono::time_point<std::chrono::high_resolution_clock>
      previous_measurement_time_;          ///< Previous sensor measurement time
  bool previous_measurements_initialized_; ///< Flag to determin whether
                                           /// previous measurements have been
                                           /// initialized
  std::queue<Eigen::Vector3d> rpy_command_buffer_; ///< Rpy command buffer
  Eigen::Vector3d filtered_rpydot_;                ///< Filtered rpydot
  Eigen::Vector3d filtered_velocity_;              ///< Filtered rpydot
  int delay_buffer_size_; ///< Size of rpy command buffer
  AbstractMPCController<StateType, ControlType>
      &private_controller_; ///< Private ref
  boost::mutex
      copy_mutex_; ///< Mutex for copying states and reference trajectories
  bool use_perfect_time_diff_; ///< Flag to use perfect time diff when
                               /// differentiating
  double perfect_time_diff_;   ///< The absolute time difference
  double angular_exp_gain_;    ///< Exponential gain
  double velocity_exp_gain_;   ///< Exponential gain
};
