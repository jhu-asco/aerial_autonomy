#include <aerial_autonomy/estimators/thrust_gain_estimator.h>
#include <aerial_autonomy/tests/test_utils.h>
#include <glog/logging.h>
#include <tf/tf.h>

#include <gtest/gtest.h>

TEST(ThrustGainEstimatorTests, Constructor) {
  ASSERT_NO_THROW(ThrustGainEstimator(0.16, 0.1));
  ASSERT_DEATH(ThrustGainEstimator(-1, 0.5),
               "Thrust gain should be greater than or equal to minimum: 0.1");
  ASSERT_DEATH(ThrustGainEstimator(5, 0.5),
               "Thrust gain should be less than or equal to maximum: 0.25");
  ASSERT_DEATH(ThrustGainEstimator(0.16, -0.5),
               "Mixing gain should be between 0 and 1");
  ASSERT_DEATH(ThrustGainEstimator(0.16, 1.5),
               "Mixing gain should be between 0 and 1");
  ASSERT_DEATH(ThrustGainEstimator(0.16, 0.5, 0),
               "Buffer size should be atleast 1");
}

TEST(ThrustGainEstimatorTests, resetThrustGain) {
  ThrustGainEstimator thrust_gain_estimator(0.16);
  ASSERT_EQ(thrust_gain_estimator.getThrustGain(), 0.16);
  thrust_gain_estimator.resetThrustGain(0.2);
  ASSERT_EQ(thrust_gain_estimator.getThrustGain(), 0.2);
  ASSERT_DEATH(thrust_gain_estimator.resetThrustGain(0);
               , "Thrust gain should be greater than or equal to minimum: 0.1");
  ASSERT_DEATH(thrust_gain_estimator.resetThrustGain(5);
               , "Thrust gain should be less than or equal to maximum: 0.25");
}

TEST(ThrustGainEstimatorTests, processSensorThrustPairZeroRollPitch) {
  double thrust_gain = 0.16;
  ThrustGainEstimator thrust_gain_estimator(thrust_gain);
  double thrust_command = 80;
  double body_z_acc = thrust_command * thrust_gain - 9.81;
  ASSERT_NEAR(thrust_gain_estimator.processSensorThrustPair(0, 0, body_z_acc,
                                                            thrust_command),
              thrust_gain, 1e-8);
}

TEST(ThrustGainEstimatorTests, processSensorZeroThrustPairZeroRollPitch) {
  double thrust_gain = 0.16;
  ThrustGainEstimator thrust_gain_estimator(thrust_gain);
  double thrust_command = 0;
  double body_z_acc = thrust_command * thrust_gain - 9.81;
  ASSERT_NEAR(thrust_gain_estimator.processSensorThrustPair(0, 0, body_z_acc,
                                                            thrust_command),
              0, 1e-8);
}

TEST(ThrustGainEstimatorTests, processSensorThrustPair90Pitch) {
  double thrust_gain = 0.16;
  ThrustGainEstimator thrust_gain_estimator(thrust_gain);
  double thrust_command = 50;
  double body_z_acc = thrust_command * thrust_gain;
  ASSERT_NEAR(thrust_gain_estimator.processSensorThrustPair(
                  M_PI / 2.0, 0, body_z_acc, thrust_command),
              thrust_gain, 1e-8);
  ASSERT_NEAR(thrust_gain_estimator.processSensorThrustPair(
                  0, M_PI / 2.0, body_z_acc, thrust_command),
              thrust_gain, 1e-8);
}

TEST(ThrustGainEstimatorTests, addThrustCommand) {
  int buffer_size = 5;
  ThrustGainEstimator thrust_gain_estimator(0.16, 0.1, buffer_size);
  ASSERT_EQ(thrust_gain_estimator.getQueueSize(), 0);
  for (int i = 0; i < 5; ++i) {
    thrust_gain_estimator.addThrustCommand(20);
  }
  ASSERT_EQ(thrust_gain_estimator.getQueueSize(), 5);
  thrust_gain_estimator.addThrustCommand(10);
  ASSERT_EQ(thrust_gain_estimator.getQueueSize(), 5);
}

TEST(ThrustGainEstimatorTests, testConvergence) {
  ThrustGainEstimator thrust_gain_estimator(0.16, 0.1);
  double t = 0;
  double dt = 0.02;
  double thrust_gain = 0.1;
  auto runEstimator = [&]() {
    double roll = cos(t);
    double pitch = sin(t);
    double thrust_command =
        9.81 / thrust_gain_estimator.getThrustGain() + 5 * sin(t);
    tf::Transform body_rotation(tf::createQuaternionFromRPY(roll, pitch, 0));
    t += dt;
    tf::Vector3 global_acceleration =
        thrust_gain * thrust_command * (body_rotation * tf::Vector3(0, 0, 1)) +
        tf::Vector3(0, 0, -9.81);
    tf::Vector3 body_acceleration =
        body_rotation.inverse() * global_acceleration;
    thrust_gain_estimator.addSensorData(roll, pitch, body_acceleration.z());
    thrust_gain_estimator.addThrustCommand(thrust_command);
    double estimated_thrust_gain = thrust_gain_estimator.getThrustGain();
    return std::abs(estimated_thrust_gain - thrust_gain) < 1e-5;
  };
  ASSERT_TRUE(test_utils::waitUntilTrue()(runEstimator, std::chrono::seconds(5),
                                          std::chrono::milliseconds(0)));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
