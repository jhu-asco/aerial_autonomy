#pragma once
#include "thrust_gain_estimator_config.pb.h"
#include <queue>

/**
 * @brief Estimates the thrust gain given body acceleration, roll, pitch and
 * commanded thrust
 */
class ThrustGainEstimator {
public:
  /**
   * @brief Estimates thrust gain given current roll, pitch, body z acceleration
   * and thrust commands
   *
   * Usage:
   *      ThrustGainEstimator thrust_gain_estimator();
   *      thrust_gain_estimator.addSensorData(current_roll, current_pitch,
   * current_body_z_acc);
   *      thrust_gain_estimator.addThrustCommand();
   *      double updated_gain = thrust_gain_estimator.getThrustGain();
   *
   * @param thrust_estimator_config specify settings for estimator i.e
   *      the initial thrust gain, buffer size, mixing gain.
   */
  ThrustGainEstimator(
      ThrustGainEstimatorConfig config = ThrustGainEstimatorConfig())
      : ThrustGainEstimator(config.kt(), config.mixing_gain(),
                            config.buffer_size(), config.kt_max(),
                            config.kt_min()) {}

  /**
   * @brief Estimates thrust gain given current roll, pitch, body z acceleration
   * and thrust commands
   *
   * Usage:
   *      ThrustGainEstimator thrust_gain_estimator(0.16);
   *      thrust_gain_estimator.addSensorData(current_roll, current_pitch,
   * current_body_z_acc);
   *      thrust_gain_estimator.addThrustCommand();
   *      double updated_gain = thrust_gain_estimator.getThrustGain();
   *
   * @param thrust_gain_initial Initial thrust gain should be greater than 0
   * @param mixing_gain Gain to update the internal thrust gain based on
   * estimated thrust gain.
   *                    If the mixing gain is close to 1, the internal gain will
   * follow the estimated
   *                    thrust gain closely. The mixing gain has to be between 0
   * and 1
   * @param buffer_size The buffer size specifies the delay between thrust
   * commands and sensor data.
   *                    A buffer size of 1 implies that there is no delay and
   * the sensor data from last
   *                    step is used to process the sensor data.
   */
  ThrustGainEstimator(double thrust_gain_initial, double mixing_gain = 0.1,
                      unsigned int buffer_size = 1,
                      double max_thrust_gain = 0.25,
                      double min_thrust_gain = 0.1);
  /**
   * @brief reset internal thrust gain to a specified value
   * This is to reset the estimator in case of failue in learning. If the
   * specified
   * thrust gain is less than zero, will throw an error.
   *
   * @param thrust_gain gain to reset to.
   */
  void resetThrustGain(double thrust_gain);
  /**
   * @brief process sensor data with the last available thrust command and
   * update internal thrust gain
   * Will only process sensor data if the thrust command queue is full.
   * Otherwise waits for enough
   * thrust commands to be filled up.
   *
   * @param roll current roll in radians under Euler ZYX convention
   * @param pitch current pitch in radians under Euler ZYX convention
   * @param body_z_acc Current body z acceleration in meters per second square
   * (accelerometer reading)
   */
  void addSensorData(double roll, double pitch, double body_z_acc);
  /**
   * @brief add thrust command to queue.
   * If queue is full, the command in front is discarded to mantain buffer size
   *
   * @param thrust_command the thrust command to add
   */
  void addThrustCommand(double thrust_command);
  /**
   * @brief get current thrust gain
   * @return  current thrust gain
   */
  double getThrustGain();
  /**
   * @brief process sensor data and thrust command to return estimated gain
   *
   * @param roll current roll in radians under Euler ZYX convention
   * @param pitch current pitch in radians under Euler ZYX convention
   * @param body_z_acc Current body z acceleration in meters per second square
   * (accelerometer reading)
   * @param thrust_command Commanded thrust in its own units
   * @return estimated thrust gain
   */
  double processSensorThrustPair(double roll, double pitch, double body_z_acc,
                                 double thrust_command);
  /**
   * @brief getQueueSize
   * @return he internal thrust command queue size
   */
  int getQueueSize();

private:
  std::queue<double> thrust_command_queue_;
  double thrust_gain_;
  double mixing_gain_;
  unsigned int delay_buffer_size_;
  const double gravity_magnitude_;
  const double thrust_command_tolerance_;
  const double max_thrust_gain_;
  const double min_thrust_gain_;
};
