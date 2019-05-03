#include "aerial_autonomy/estimators/acceleration_bias_estimator.h"
#include "aerial_autonomy/common/conversions.h"
#include <aerial_autonomy/common/math.h>
#include <glog/logging.h>

AccelerationBiasEstimator::AccelerationBiasEstimator(
    AccelerationBiasEstimatorConfig config)
    : AccelerationBiasEstimator(config.mixing_gain(), config.max_bias(),
                                config.delay_buffer_size()) {}

AccelerationBiasEstimator::AccelerationBiasEstimator(
    double mixing_gain, double max_bias, unsigned int delay_buffer_size)
    : acceleration_bias_filter_(mixing_gain), max_bias_(max_bias),
      delay_buffer_size_(delay_buffer_size), gravity_magnitude_(9.81) {
  CHECK_GE(max_bias_, 0) << "Max bias should be non-negative";
  CHECK_GE(delay_buffer_size_, 1) << "Buffer size should be atleast 1";
  acceleration_bias_filter_.setFilterData(Eigen::Vector3d(0, 0, 0));
}

void AccelerationBiasEstimator::reset() {
  acceleration_bias_filter_.reset();
  acceleration_bias_filter_.setFilterData(Eigen::Vector3d(0, 0, 0));

  std::queue<double> empty;
  acceleration_commands_.swap(empty);
}

Eigen::Vector3d AccelerationBiasEstimator::computeAccelerationBias(
    double roll, double pitch, Eigen::Vector3d body_acc, double thrust_acc) {
  Eigen::Matrix3d rotation;
  conversions::transformRPYToMatrix3d(roll, pitch, 0, rotation);
  Eigen::Vector3d gravity(0, 0, -gravity_magnitude_);
  Eigen::Vector3d gravity_body = rotation.transpose() * gravity;

  return body_acc - gravity_body - Eigen::Vector3d(0, 0, thrust_acc);
}

void AccelerationBiasEstimator::addSensorData(double roll, double pitch,
                                              Eigen::Vector3d body_acc) {
  if (acceleration_commands_.size() == delay_buffer_size_) {
    auto acc_bias = computeAccelerationBias(roll, pitch, body_acc,
                                            acceleration_commands_.front());
    acceleration_bias_filter_.add(acc_bias);

    // Clamp bias
    auto filtered_bias = acceleration_bias_filter_.getFilterData();
    for (unsigned int i = 0; i < 3; i++) {
      filtered_bias[i] = math::clamp(filtered_bias[i], -max_bias_, max_bias_);
    }
    acceleration_bias_filter_.setFilterData(filtered_bias);
  } else {
    VLOG(2) << "Waiting for thrust commands to fill up the buffer";
  }
}

void AccelerationBiasEstimator::addAccelerationCommand(double acc) {
  if (acceleration_commands_.size() == delay_buffer_size_) {
    acceleration_commands_.pop();
  }
  acceleration_commands_.push(acc);
}

Eigen::Vector3d AccelerationBiasEstimator::getAccelerationBias() const {
  if (acceleration_bias_filter_.isDataAvailable()) {
    return acceleration_bias_filter_.getFilterData();
  } else {
    return Eigen::Vector3d(0, 0, 0);
  }
}
