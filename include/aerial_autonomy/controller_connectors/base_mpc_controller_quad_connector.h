#pragma once
#include "aerial_autonomy/controller_connectors/mpc_controller_connector.h"
#include "aerial_autonomy/estimators/thrust_gain_estimator.h"
#include "aerial_autonomy/filters/exponential_filter.h"
#include "aerial_autonomy/sensors/base_sensor.h"
#include "mpc_connector_config.pb.h"
#include <Eigen/Dense>
#include <chrono>
#include <parsernode/parser.h>
#include <queue>
#include <tf/tf.h>

/**
* @brief Controller connector common code between airm and quadrotor
* system
*/
class BaseMPCControllerQuadConnector
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
  BaseMPCControllerQuadConnector(
      parsernode::Parser &drone_hardware,
      AbstractMPCController<StateType, ControlType> &controller,
      ThrustGainEstimator &thrust_gain_estimator, int delay_buffer_size,
      MPCConnectorConfig config,
      SensorPtr<std::pair<tf::StampedTransform, tf::Vector3>> odom_sensor =
          nullptr,
      AbstractConstraintGeneratorPtr constraint_generator = nullptr);

  /**
   * @brief ~BaseMPCControllerQuadConnector Destructor
   */
  virtual ~BaseMPCControllerQuadConnector() {}

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
  void
  useSensor(SensorPtr<std::pair<tf::StampedTransform, tf::Vector3>> sensor);

protected:
  /**
  * @brief Set rpy command buffer to zeros
  */
  void clearCommandBuffers();

  /**
  * @brief Estimate the current state and static MPC parameters
  *
  * for quadrotor
  *
  * @param current_state Current system state
  * @param params Current MPC parameters
  *
  * @return true if estimation is successful
  */
  bool fillQuadStateAndParameters(Eigen::VectorXd &current_state,
                                  Eigen::VectorXd &params);

  /**
   * @brief initialize the controller
   *
   * @tparam T  The type of controller
   * @param private_controller private controller
   */
  template <class T> void initializePrivateController(T &private_controller) {
    MPCControllerConnector::initialize();
    VLOG(1) << "Clearing thrust estimator buffer";
    thrust_gain_estimator_.clearBuffer();
    private_controller.resetControls();
    clearCommandBuffers();
    rpydot_filter_.reset();
    int iters = private_controller.getMaxIters();
    private_controller.setMaxIters(100);
    run();
    private_controller.setMaxIters(iters);
  }

protected:
  parsernode::Parser
      &drone_hardware_; ///< Quadrotor parser for sending and receiving data
                        /**
                         * @brief Pose sensor for quad data
                         */
  SensorPtr<std::pair<tf::StampedTransform, tf::Vector3>> odom_sensor_;
  ThrustGainEstimator &thrust_gain_estimator_;       ///< Thrust gain estimator
  std::queue<Eigen::Vector3d> rpy_command_buffer_;   ///< Rpy command buffer
  ExponentialFilter<Eigen::Vector3d> rpydot_filter_; ///< Filter
  int delay_buffer_size_; ///< Size of rpy command buffer
  AbstractMPCController<StateType, ControlType>
      &private_controller_; ///< Private ref
  boost::mutex
      copy_mutex_; ///< Mutex for copying states and reference trajectories
  MPCConnectorConfig config_; ///< Config for mpc connector
};
