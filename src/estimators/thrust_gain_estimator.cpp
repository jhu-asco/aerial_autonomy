#include <aerial_autonomy/common/conversions.h>
#include <aerial_autonomy/common/math.h>
#include <aerial_autonomy/estimators/thrust_gain_estimator.h>
#include <aerial_autonomy/log/log.h>
#include <glog/logging.h>
#include <math.h>

ThrustGainEstimator::ThrustGainEstimator(
    double thrust_gain_initial, double mixing_gain, unsigned int buffer_size,
    double max_thrust_gain, double min_thrust_gain, double max_roll_pitch_bias,
    double rp_mixing_gain, double init_roll_bias, double init_pitch_bias)
    : thrust_gain_(thrust_gain_initial),
      roll_pitch_bias_(init_roll_bias, init_pitch_bias),
      mixing_gain_(mixing_gain), config_mixing_gain_(mixing_gain),
      rp_mixing_gain_(rp_mixing_gain), delay_buffer_size_(buffer_size),
      gravity_magnitude_(9.81), thrust_command_tolerance_(1e-2),
      max_thrust_gain_(max_thrust_gain), min_thrust_gain_(min_thrust_gain),
      max_roll_pitch_bias_(max_roll_pitch_bias),
      init_roll_bias_(init_roll_bias), init_pitch_bias_(init_pitch_bias) {
  CHECK_GE(delay_buffer_size_, 1) << "Buffer size should be atleast 1";
  CHECK_GE(mixing_gain_, 0) << "Mixing gain should be between 0 and 1";
  CHECK_LE(mixing_gain_, 1) << "Mixing gain should be between 0 and 1";
  CHECK_GT(rp_mixing_gain_, 0) << "Mixing gain should be between 0 and 1";
  CHECK_LT(rp_mixing_gain_, 1) << "Mixing gain should be between 0 and 1";
  CHECK_GE(thrust_gain_initial, min_thrust_gain_)
      << "Thrust gain should be greater than or equal to minimum: "
      << min_thrust_gain_;
  CHECK_LE(thrust_gain_initial, max_thrust_gain_)
      << "Thrust gain should be less than or equal to maximum: "
      << max_thrust_gain_;
  DATA_HEADER("thrust_gain_estimator") << "roll"
                                       << "pitch"
                                       << "body_x_acc"
                                       << "body_y_acc"
                                       << "body_z_acc"
                                       << "thrust_command"
                                       << "thrust_gain"
                                       << "roll_bias"
                                       << "pitch_bias" << DataStream::endl;
  //std::cout << "TGE Constructor" << std::endl;
}

void ThrustGainEstimator::reset(double thrust_gain) {
  //std::cout << "TGE Reset" << std::endl;
  CHECK_GE(thrust_gain, min_thrust_gain_)
      << "Thrust gain should be greater than or equal to minimum: "
      << min_thrust_gain_;
  CHECK_LE(thrust_gain, max_thrust_gain_)
      << "Thrust gain should be less than or equal to maximum: "
      << max_thrust_gain_;
  thrust_gain_ = thrust_gain;
  roll_pitch_bias_ = Eigen::Vector2d(init_roll_bias_, init_pitch_bias_);
}

std::pair<double, Eigen::Vector2d> ThrustGainEstimator::processSensorThrustPair(
    double roll, double pitch, tf::Vector3 body_acc, double thrust_command) {
  //std::cout << "TGE process ST pair" << std::endl;
  tf::Transform quad_rotation(tf::createQuaternionFromRPY(roll, pitch, 0));
  tf::Vector3 rotated_acc = quad_rotation * body_acc;
  rotated_acc[2] = rotated_acc[2] + gravity_magnitude_;
  double thrust_command_clipped =
      std::max(thrust_command_tolerance_, thrust_command);
  double gravity_z_acc_comp = gravity_magnitude_ * cos(roll) * cos(pitch);
  double thrust_gain_estimated =
      (body_acc[2] + gravity_z_acc_comp) / thrust_command_clipped;
  // double thrust_gain_estimated = rotated_acc.length() /
  // thrust_command_clipped;
  Eigen::Vector3d rotated_acc_eig(rotated_acc[0], rotated_acc[1],
                                  rotated_acc[2]);
  auto roll_pitch = conversions::accelerationToRollPitch(0, rotated_acc_eig);
  return std::make_pair(
      thrust_gain_estimated,
      Eigen::Vector2d(roll_pitch.first - roll, roll_pitch.second - pitch));
}

int ThrustGainEstimator::getQueueSize() { return thrust_command_queue_.size(); }

void ThrustGainEstimator::addSensorData(double roll, double pitch,
                                        tf::Vector3 body_acc) {
  //std::cout << "TGE addSensorData" << std::endl;

  if (thrust_command_queue_.size() == delay_buffer_size_) {
    auto gain_bias_pair = processSensorThrustPair(
        roll, pitch, body_acc, thrust_command_queue_.front());
    double &thrust_gain_estimated = gain_bias_pair.first;
    thrust_gain_ = mixing_gain_ * thrust_gain_estimated +
                   (1 - mixing_gain_) * thrust_gain_;
    thrust_gain_ =
        math::clamp(thrust_gain_, min_thrust_gain_, max_thrust_gain_);
    Eigen::Vector2d &roll_pitch_bias_estimated = gain_bias_pair.second;
    roll_pitch_bias_ = rp_mixing_gain_ * roll_pitch_bias_estimated +
                       (1 - rp_mixing_gain_) * roll_pitch_bias_;
    roll_pitch_bias_[0] = math::clamp(
        roll_pitch_bias_[0], -max_roll_pitch_bias_, max_roll_pitch_bias_);
    roll_pitch_bias_[1] = math::clamp(
        roll_pitch_bias_[1], -max_roll_pitch_bias_, max_roll_pitch_bias_);
    DATA_LOG("thrust_gain_estimator")
        << roll << pitch << body_acc[0] << body_acc[1] << body_acc[2]
        << thrust_command_queue_.front() << thrust_gain_ << roll_pitch_bias_
        << DataStream::endl;
  } else {
    VLOG(2) << "Waiting for thrust commands to fill up the buffer";
  }
}

void ThrustGainEstimator::addThrustCommand(double thrust_command) {
  //std::cout << "TGE addThrust" << std::endl;
  if (thrust_command_queue_.size() == delay_buffer_size_) {
    thrust_command_queue_.pop();
  }
  thrust_command_queue_.push(thrust_command);
}

double ThrustGainEstimator::getThrustGain() { 
  //std::cout << "TGE get Thrust Gain" << std::endl;
  return thrust_gain_; 
}

Eigen::Vector2d ThrustGainEstimator::getRollPitchBias() {
  //std::cout << "TGE get RP Bias" << std::endl;
  return roll_pitch_bias_;
}

void ThrustGainEstimator::clearBuffer() {
  //std::cout << "TGE clear Buffer" << std::endl;
  std::queue<double> empty;
  thrust_command_queue_.swap(empty);
}

void ThrustGainEstimator::resetThrustMixingGain() {
  //std::cout << "TGE reset Mixing Gain" << std::endl;
  mixing_gain_ = config_mixing_gain_;
}

void ThrustGainEstimator::setThrustMixingGain(double mixing_gain) {
  //std::cout << "TGE set Mixing Gain" << std::endl;
  CHECK_GE(mixing_gain, 0) << "Mixing gain should be between 0 and 1";
  CHECK_LE(mixing_gain, 1) << "Mixing gain should be between 0 and 1";
  mixing_gain_ = mixing_gain;
}
