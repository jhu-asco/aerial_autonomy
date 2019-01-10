#include "aerial_autonomy/estimators/acceleration_bias_estimator.h"
#include "aerial_autonomy/tests/test_utils.h"
#include <gtest/gtest.h>

using namespace test_utils;

class AccelerationBiasEstimatorTests : public ::testing::Test {
public:
  AccelerationBiasEstimatorTests() : estimator_(0.5, 1.0, 10) {}

protected:
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
  for (int i = 0; i < 10; i++) {
    ASSERT_VEC_NEAR(Eigen::Vector3d(0, 0, 0), estimator_.getAccelerationBias());
    estimator_.addAccelerationCommand(0.1);
    estimator_.addSensorData(0, 0, Eigen::Vector3d(0, 0, 9.81));
  }
  ASSERT_VEC_NEAR(Eigen::Vector3d(0, 0, -0.1),
                  estimator_.getAccelerationBias());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
