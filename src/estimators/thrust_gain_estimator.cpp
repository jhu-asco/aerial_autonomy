#include <aerial_autonomy/estimators/thrust_gain_estimator.h>
#include <glog/logging.h>
#include <math.h>

ThrustGainEstimator::ThrustGainEstimator(double thrust_gain_initial,
                                         double mixing_gain,
                                         unsigned int buffer_size)
    : thrust_gain_(thrust_gain_initial), mixing_gain_(mixing_gain),
      buffer_size_(buffer_size), gravity_magnitude_(9.81),
      thrust_command_tolerance_(1e-2) {
  CHECK_GE(buffer_size_, 1) << "Buffer size should be atleast 1";
  CHECK_GT(mixing_gain_, 0) << "Mixing gain should be between 0 and 1";
  CHECK_LT(mixing_gain_, 1) << "Mixing gain should be between 0 and 1";
  CHECK_GT(thrust_gain_initial, 0) << "Thrust gain should be greater than 0";
}

void ThrustGainEstimator::resetThrustGain(double thrust_gain) {
  CHECK_GT(thrust_gain, 0) << "Thrust gain should be greater than 0";
  thrust_gain_ = thrust_gain;
}

double ThrustGainEstimator::processSensorThrustPair(double roll, double pitch,
                                                    double body_z_acc,
                                                    double thrust_command) {
  double gravity_z_acc_comp = gravity_magnitude_ * cos(roll) * cos(pitch);
  double thrust_command_clipped =
      std::max(thrust_command_tolerance_, thrust_command);
  return (body_z_acc + gravity_z_acc_comp) / thrust_command_clipped;
}

int ThrustGainEstimator::getQueueSize() { return thrust_command_queue_.size(); }

void ThrustGainEstimator::addSensorData(double roll, double pitch,
                                        double body_z_acc) {
  if (thrust_command_queue_.size() == buffer_size_) {
    double thrust_gain_estimated = processSensorThrustPair(
        roll, pitch, body_z_acc, thrust_command_queue_.front());
    thrust_gain_ = mixing_gain_ * thrust_gain_estimated +
                   (1 - mixing_gain_) * thrust_gain_;
  } else {
    VLOG(2) << "Waiting for thrust commands to fill up the buffer";
  }
}

void ThrustGainEstimator::addThrustCommand(double thrust_command) {
  if (thrust_command_queue_.size() == buffer_size_) {
    thrust_command_queue_.pop();
  }
  thrust_command_queue_.push(thrust_command);
}

double ThrustGainEstimator::getThrustGain() { return thrust_gain_; }
