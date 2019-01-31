#include "aerial_autonomy/estimators/acceleration_bias_estimator.h"
#include "aerial_autonomy/tests/test_utils.h"
#include <gtest/gtest.h>

using namespace test_utils;

class AccelerationBiasEstimatorTests : public ::testing::Test {
public:
  AccelerationBiasEstimatorTests()
      : buffer_size_(10), max_bias_(1.0), mixing_factor_(0.5),
        estimator_(mixing_factor_, max_bias_, buffer_size_) {}

protected:
  void fillBuffer(AccelerationBiasEstimator &estimator,
                  unsigned int buffer_size, double roll, double pitch,
                  double acc) {
    for (unsigned int i = 0; i < buffer_size; i++) {
      ASSERT_VEC_NEAR(Eigen::Vector3d(0, 0, 0),
                      estimator.getAccelerationBias());
      estimator.addAccelerationCommand(acc);
      estimator.addSensorData(roll, pitch, Eigen::Vector3d(0, 0, 0));
    }
  }

  const unsigned int buffer_size_;
  double max_bias_;
  double mixing_factor_;
  AccelerationBiasEstimator estimator_;
};

TEST_F(AccelerationBiasEstimatorTests, Constructor) {
  ASSERT_NO_THROW(AccelerationBiasEstimator());
  ASSERT_NO_THROW(AccelerationBiasEstimator(0.5, 1.0, 1));

  ASSERT_DEATH(AccelerationBiasEstimator(-1, 1.0, 1), "");
  ASSERT_DEATH(AccelerationBiasEstimator(0.5, -1, 1),
               "Max bias should be non-negative");
  ASSERT_DEATH(AccelerationBiasEstimator(0.5, 1, 0),
               "Buffer size should be atleast 1");
}

TEST_F(AccelerationBiasEstimatorTests, NoRollPitch) {
  fillBuffer(estimator_, buffer_size_, 0, 0, 9.91);
  ASSERT_VEC_NEAR(Eigen::Vector3d(0, 0, -0.1 * mixing_factor_),
                  estimator_.getAccelerationBias());
}

TEST_F(AccelerationBiasEstimatorTests, SmallDelayBuffer) {
  AccelerationBiasEstimator estimator(0.5, 1.0, 1);
  fillBuffer(estimator, 1, 0, 0, 9.91);
  ASSERT_VEC_NEAR(Eigen::Vector3d(0, 0, -0.1 * mixing_factor_),
                  estimator.getAccelerationBias());
}

TEST_F(AccelerationBiasEstimatorTests, MaxBias) {
  fillBuffer(estimator_, buffer_size_, 0, 0, 11.81);
  ASSERT_VEC_NEAR(Eigen::Vector3d(0, 0, -max_bias_),
                  estimator_.getAccelerationBias());
}

TEST_F(AccelerationBiasEstimatorTests, MaxBiasNeg) {
  fillBuffer(estimator_, buffer_size_, 0, 0, 7.81);
  ASSERT_VEC_NEAR(Eigen::Vector3d(0, 0, max_bias_),
                  estimator_.getAccelerationBias());
}

TEST_F(AccelerationBiasEstimatorTests, MaxBiasWithPitch) {
  fillBuffer(estimator_, buffer_size_, 0, 5. * M_PI / 180., 11.81);
  ASSERT_VEC_NEAR(
      Eigen::Vector3d(-sin(5. * M_PI / 180.) * 9.81 * mixing_factor_, 0,
                      -max_bias_),
      estimator_.getAccelerationBias());
}

TEST_F(AccelerationBiasEstimatorTests, MaxBiasWithNegPitch) {
  fillBuffer(estimator_, buffer_size_, 0, -5. * M_PI / 180., 11.81);
  ASSERT_VEC_NEAR(Eigen::Vector3d(sin(5. * M_PI / 180.) * 9.81 * mixing_factor_,
                                  0, -max_bias_),
                  estimator_.getAccelerationBias());
}

TEST_F(AccelerationBiasEstimatorTests, MaxBiasWithRoll) {
  fillBuffer(estimator_, buffer_size_, 5. * M_PI / 180., 0, 11.81);
  ASSERT_VEC_NEAR(Eigen::Vector3d(0,
                                  sin(5. * M_PI / 180.) * 9.81 * mixing_factor_,
                                  -max_bias_),
                  estimator_.getAccelerationBias());
}

TEST_F(AccelerationBiasEstimatorTests, MaxBiasAllClamped) {
  fillBuffer(estimator_, buffer_size_, 45. * M_PI / 180., 45. * M_PI / 180,
             100);
  ASSERT_VEC_NEAR(Eigen::Vector3d(-max_bias_, max_bias_, -max_bias_),
                  estimator_.getAccelerationBias());
}

TEST_F(AccelerationBiasEstimatorTests, DelayAndMixing) {
  double gravity = 9.81;
  double start_bias = 0.1;
  double start_cmd = gravity + start_bias;
  double buffer_bias = 0.2;
  double buffer_cmd = gravity + buffer_bias;
  estimator_.addAccelerationCommand(start_cmd);
  estimator_.addSensorData(0, 0, Eigen::Vector3d(0, 0, 0));
  fillBuffer(estimator_, buffer_size_ - 1, 0, 0, buffer_cmd);
  ASSERT_VEC_NEAR(Eigen::Vector3d(0, 0, -start_bias * mixing_factor_),
                  estimator_.getAccelerationBias());

  double z_bias = mixing_factor_ * start_bias +
                  mixing_factor_ * (buffer_bias - mixing_factor_ * start_bias);
  estimator_.addAccelerationCommand(start_cmd);
  estimator_.addSensorData(0, 0, Eigen::Vector3d(0, 0, 0));
  ASSERT_VEC_NEAR(Eigen::Vector3d(0, 0, -z_bias),
                  estimator_.getAccelerationBias());

  estimator_.addAccelerationCommand(start_cmd);
  estimator_.addSensorData(0, 0, Eigen::Vector3d(0, 0, 0));
  ASSERT_VEC_NEAR(
      Eigen::Vector3d(0, 0,
                      -(z_bias + mixing_factor_ * (buffer_bias - z_bias))),
      estimator_.getAccelerationBias());
}

TEST_F(AccelerationBiasEstimatorTests, Reset) {
  fillBuffer(estimator_, buffer_size_, 0, 0, 9.91);
  ASSERT_VEC_NEAR(Eigen::Vector3d(0, 0, -0.1 * mixing_factor_),
                  estimator_.getAccelerationBias());

  estimator_.reset();

  fillBuffer(estimator_, buffer_size_, 0, 0, 9.91);
  ASSERT_VEC_NEAR(Eigen::Vector3d(0, 0, -0.1 * mixing_factor_),
                  estimator_.getAccelerationBias());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
