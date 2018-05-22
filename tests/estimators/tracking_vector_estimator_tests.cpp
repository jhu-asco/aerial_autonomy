#include <aerial_autonomy/estimators/tracking_vector_estimator.h>
#include <aerial_autonomy/log/log.h>
#include <aerial_autonomy/tests/test_utils.h>
#include <glog/logging.h>
#include <tf/tf.h>

#include <gtest/gtest.h>

using namespace test_utils;

class TrackingVectorEstimatorTests : public ::testing::Test {
public:
  static void SetUpTestCase() {
    // Configure logging
    LogConfig log_config;
    log_config.set_directory("/tmp/data");
    Log::instance().configure(log_config);
    DataStreamConfig data_config;
    data_config.set_stream_id("tracking_vector_estimator");
    Log::instance().addDataStream(data_config);
  }

protected:
  TrackingVectorEstimatorConfig config_;
};

TEST_F(TrackingVectorEstimatorTests, Constructor) {
  ASSERT_NO_THROW(
      TrackingVectorEstimator(config_, std::chrono::milliseconds(20)));
  config_.mutable_marker_process_stdev()->set_x(-1);
  ASSERT_DEATH(TrackingVectorEstimator(config_, std::chrono::milliseconds(20)),
               "Stdev vector should be greater than zero tolerance");
}

TEST_F(TrackingVectorEstimatorTests, initializeState) {
  tf::Vector3 initial_marker_direction(1, 0, 0);
  config_.mutable_marker_initial_stdev()->set_x(1);
  TrackingVectorEstimator estimator(config_, std::chrono::milliseconds(20));
  estimator.initializeState(initial_marker_direction);
  ASSERT_VEC_NEAR(estimator.getMarkerDirection(), initial_marker_direction);
  ASSERT_VEC_NEAR(estimator.getMarkerNoise(), tf::Vector3(1, 1e-2, 1e-2));
}

TEST_F(TrackingVectorEstimatorTests, testMeasurementCovariance) {
  TrackingVectorEstimator estimator(config_, std::chrono::milliseconds(20));
  int seconds_offset = 1;
  auto marker_time_stamp = std::chrono::high_resolution_clock::now() -
                           std::chrono::seconds(seconds_offset);
  cv::Mat measurement_covariance_matrix;
  estimator.setMeasurementCovariance(measurement_covariance_matrix,
                                     marker_time_stamp);
  tf::Vector3 marker_meas_stdev(config_.marker_meas_stdev().x(),
                                config_.marker_meas_stdev().y(),
                                config_.marker_meas_stdev().z());
  tf::Vector3 marker_dilation_stdev(config_.marker_dilation_stdev().x(),
                                    config_.marker_dilation_stdev().y(),
                                    config_.marker_dilation_stdev().z());
  for (int i = 0; i < 3; ++i) {
    double expected_stdev =
        seconds_offset * marker_dilation_stdev[i] + marker_meas_stdev[i];
    double expected_covariance = expected_stdev * expected_stdev;
    ASSERT_NEAR(measurement_covariance_matrix.at<double>(i, i),
                expected_covariance, 1e-4);
  }
}

TEST_F(TrackingVectorEstimatorTests, predictState) {
  double dt = 0.02;
  tf::Vector3 initial_marker_direction(1, 0, 0);
  tf::Vector3 velocity(0, 1, 0);
  TrackingVectorEstimator estimator(config_, std::chrono::duration<double>(dt));
  estimator.initializeState(initial_marker_direction);
  tf::Vector3 initial_marker_noise = estimator.getMarkerNoise();
  estimator.predict(velocity);
  tf::Vector3 expected_marker_direction =
      initial_marker_direction - velocity * dt;
  ASSERT_VEC_NEAR(estimator.getPredictedMarkerDirection(),
                     expected_marker_direction);
  tf::Vector3 marker_noise = estimator.getMarkerNoise();
  for (int i = 0; i < 3; ++i) {
    ASSERT_GT(marker_noise[i], initial_marker_noise[i]);
  }
}

TEST_F(TrackingVectorEstimatorTests, correctState) {
  // Simulate Quad moving in a circle with the marker in the center
  double tol = 1e-2;
  double t = 0;
  double dt = 0.02;
  TrackingVectorEstimator estimator(config_, std::chrono::duration<double>(dt));
  estimator.initializeState(tf::Vector3(0, 0, 0));
  auto runEstimator = [&]() {
    t += dt;
    tf::Vector3 quad_pos(cos(t), sin(t), 0);
    tf::Vector3 marker_pos(0, 0, 0);
    tf::Vector3 quad_vel(-sin(t), cos(t), 0);
    tf::Vector3 marker_direction = marker_pos - quad_pos;
    estimator.predict(quad_vel);
    estimator.correct(marker_direction,
                      std::chrono::high_resolution_clock::now());
    tf::Vector3 estimated_marker_direction = estimator.getMarkerDirection();
    tf::Vector3 error_marker_direction =
        estimated_marker_direction - marker_direction;
    LOG(INFO) << marker_direction.x() << ", " << marker_direction.y() << ", "
              << marker_direction.z() << ", " << estimated_marker_direction.x()
              << ", " << estimated_marker_direction.y() << ", "
              << estimated_marker_direction.z();
    if (error_marker_direction.length() < tol)
      return true;
    return false;
  };
  ASSERT_TRUE(waitUntilTrue()(runEstimator, std::chrono::seconds(1),
                              std::chrono::milliseconds(0)));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
