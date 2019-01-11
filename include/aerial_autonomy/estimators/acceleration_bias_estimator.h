#pragma once

#include "acceleration_bias_estimator_config.pb.h"
#include "aerial_autonomy/filters/exponential_filter.h"
#include <Eigen/Dense>
#include <queue>

/**
 * @brief Computes an acceleration bias for a quadrotor that is assumed to be
 *only
 * under the influence of gravity and thrust
 **/
class AccelerationBiasEstimator {
public:
  /**
  * @brief Constructor
  * @param config Configuration
  */
  AccelerationBiasEstimator(AccelerationBiasEstimatorConfig config =
                                AccelerationBiasEstimatorConfig());
  /**
  * @brief Constructor
  * @param mixing_gain Gain of the bias filter
  * @param max_bias Max allowed value for each component of the bias
  * @param delay_buffer_size Size of buffer for delaying thrust commands
  */
  AccelerationBiasEstimator(double mixing_gain, double max_bias,
                            unsigned int delay_buffer_size);
  /**
  * @brief Resets the estimator to its initial state
  */
  void reset();
  /**
  * @brief Add sensor data to be processed by the estimator
  * @param roll Roll of the robot
  * @param pitch Pitch of the robot
  * @param body_acc body acceleration measured by an IMU
  */
  void addSensorData(double roll, double pitch, Eigen::Vector3d body_acc);
  /**
  * @brief Add current acceleration command to the estimator
  * @param acc Current Acceleration command
  */
  void addAccelerationCommand(double acc);
  /**
  * @brief Get the current acceleration bias
  * @return Acceleration bias
  */
  Eigen::Vector3d getAccelerationBias();
  /**
  * @brief Compute the acceleration bias
  * @param roll Roll of robot
  * @param pitch Pitch of robot
  * @param body_acc Body acceleration of the robot measured by IMU
  * @param thrust_acc Expected accelerationd ue to thrust
  */
  Eigen::Vector3d computeAccelerationBias(double roll, double pitch,
                                          Eigen::Vector3d body_acc,
                                          double thrust_acc);

private:
  /**
  * @brief Filter for acceleration bias
  */
  ExponentialFilter<Eigen::Vector3d> acceleration_bias_filter_;
  /**
  * @brief Maximum allowed bias
  */
  double max_bias_;
  /**
  * @brief Acceleration commands
  */
  std::queue<double> acceleration_commands_;
  /**
   * @brief Assuming quad command is at 50 Hz, the buffer size is delay between
   * when command is sent and sensor measurements are received
   */
  unsigned int delay_buffer_size_;
  /**
  * @brief Magnitude of gravity
  */
  const double gravity_magnitude_;
};
