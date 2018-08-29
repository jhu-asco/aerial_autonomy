#pragma once
#include "thrust_gain_estimator_config.pb.h"
#include <Eigen/Dense>
#include <queue>
#include <tf/tf.h>

/**
 * @brief Estimates the thrust gain, rp bias given body acceleration, roll,
 * pitch and
 * commanded thrust
 */
class ThrustGainEstimator {
public:
  /**
   * @brief Estimates thrust gain, bias given current roll, pitch, body xyz
   * accelerations
   * and thrust commands
   *
   * Usage:
   *      ThrustGainEstimator thrust_gain_estimator();
   *      thrust_gain_estimator.addSensorData(current_roll, current_pitch,
   * current_body_acc);
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
                            config.kt_min(), config.max_roll_pitch_bias(),
                            config.rp_mixing_gain()) {}

  /**
   * @brief Estimates thrust gain,bias given current roll, pitch, body z
   * acceleration
   * and thrust commands
   *
   * Usage:
   *      ThrustGainEstimator thrust_gain_estimator(0.16);
   *      thrust_gain_estimator.addSensorData(current_roll, current_pitch,
   * current_body_acc);
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
                      double min_thrust_gain = 0.1,
                      double max_roll_pitch_bias = 0.05,
                      double rp_mixing_gain = 0.1);
  /**
   * @brief reset internal thrust gain to a specified value
   * This is to reset the estimator in case of failue in learning. If the
   * specified
   * thrust gain is less than zero, will throw an error. Also set
   * rp bias to 0
   *
   * @param thrust_gain gain to reset to.
   */
  void reset(double thrust_gain);
  /**
   * @brief process sensor data with the last available thrust command and
   * update internal thrust gain, bias
   * Will only process sensor data if the thrust command queue is full.
   * Otherwise waits for enough
   * thrust commands to be filled up.
   *
   * Note: the estimator takes into account delay between thrust and
   * acceleration measurements by using a buffer. The buffer size
   * multiplied by the controller frequency should be equal to the sensor
   * delay
   *
   * @param roll current roll in radians under Euler ZYX convention
   * @param pitch current pitch in radians under Euler ZYX convention
   * @param body_acc Current body acceleration in meters per second square
   * (accelerometer reading)
   */
  void addSensorData(double roll, double pitch, tf::Vector3 body_acc);
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
   * @brief Get roll pitch bias. RPBias + roll_pitch_imu = true_rp
   *
   * @return current roll and pitch bias
   */
  Eigen::Vector2d getRollPitchBias();
  /**
   * @brief process sensor data and thrust command to return estimated gain
   *
   * @param roll current roll in radians under Euler ZYX convention
   * @param pitch current pitch in radians under Euler ZYX convention
   * @param body_acc Current body acceleration in meters per second square
   * (accelerometer reading)
   * @param thrust_command Commanded thrust in its own units
   * @return estimated thrust gain and roll pitch bias
   */
  std::pair<double, Eigen::Vector2d>
  processSensorThrustPair(double roll, double pitch, tf::Vector3 body_acc,
                          double thrust_command);
  /**
   * @brief getQueueSize
   * @return he internal thrust command queue size
   */
  int getQueueSize();

  /**
   * @brief Clear internal buffer of thrust commands. Should be called if
   * estimator
   * is stopped and is restarting after a time.
   */
  void clearBuffer();

  /**
   * @brief Set the current thrust mixing gain to specified value
   *
   * @param mixing_gain gain to set
   */
  void setThrustMixingGain(double mixing_gain);

  /**
   * @brief reset the thrust mixing gain to config specified
   */
  void resetThrustMixingGain();

private:
  /**
   * @brief Queue to store thrust commands to account for delay
   */
  std::queue<double> thrust_command_queue_;
  /**
   * @brief The gain when multiplied should match the z acceleration in body
   * frame
   */
  double thrust_gain_;
  /**
   * @brief Roll bias + roll from imu = true roll i.e ensures body acc is along
   * z axis
   */
  Eigen::Vector2d roll_pitch_bias_;
  /**
   * @brief Filtering gain used for filtering thrust gain. Should be between 0
   * and 1
   */
  double mixing_gain_;
  /**
   * @brief Filtering gain obtained from config which can be different from
   * mixing gain set using "setThrustMixingGain" function
   */
  double config_mixing_gain_;
  /**
   * @brief Filter gain for rp bias estimation
   */
  double rp_mixing_gain_;
  /**
   * @brief Assuming quad command is at 50 Hz, the buffer size is delay between
   * when command is sent and sensor measurements are received
   */
  unsigned int delay_buffer_size_;
  /**
   * @brief Magnitude of gravity is 9.81
   */
  const double gravity_magnitude_;
  /**
   * @brief Minimum thrust command allowed
   */
  const double thrust_command_tolerance_;
  /**
   * @brief Clip max thrust gain to this value
   */
  const double max_thrust_gain_;
  /**
   * @brief Clip min thrust gain to this value
   */
  const double min_thrust_gain_;
  /**
   * @brief Max rp bias allowed
   */
  const double max_roll_pitch_bias_;
};
