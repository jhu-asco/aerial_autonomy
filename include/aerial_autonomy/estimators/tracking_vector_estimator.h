#pragma once
#include "tracking_vector_estimator_config.pb.h"
#include <chrono>
#include <glog/logging.h>
#include <opencv2/video/tracking.hpp>
#include <tf/tf.h>
#include <tuple>

/**
* @brief Estimator for estimating marker direction in global frame and global
* velocity
*
* Basic Kalman filter that assumes a constant velocity model to estimate the
* marker
* direction and velocity in global frame given the measurements of the same.
*
*/
class TrackingVectorEstimator {

private:
  using StdVector3 = TrackingVectorEstimatorConfig_StdVector3;
  TrackingVectorEstimatorConfig config_;
  cv::KalmanFilter filter_;
  const double zero_tolerance_;
  bool initial_state_initialized_;
  tf::Vector3 marker_meas_stdev_;
  tf::Vector3 marker_dilation_stdev_;
  template <class T>
  void setCovarianceMatrix(cv::Mat &matrix, T marker_stdev_vector) {
    matrix = cv::Mat_<double>::zeros(3, 3);
    matrix.at<double>(0, 0) =
        (marker_stdev_vector.x() * marker_stdev_vector.x());
    matrix.at<double>(1, 1) =
        (marker_stdev_vector.y() * marker_stdev_vector.y());
    matrix.at<double>(2, 2) =
        (marker_stdev_vector.z() * marker_stdev_vector.z());
  }

  void checkStdVector(StdVector3 vec) {
    CHECK_GT(vec.x(), zero_tolerance_)
        << "Stdev vector should be greater than zero tolerance";
    CHECK_GT(vec.y(), zero_tolerance_)
        << "Stdev vector should be greater than zero tolerance";
    CHECK_GT(vec.z(), zero_tolerance_)
        << "Stdev vector should be greater than zero tolerance";
  }

public:
  /**
  * @brief Constructor
  *
  * Initializes the kalman filter with the values provided in the config.
  *
  * @param config Provide the initial state, process noise, measurement noise
  * values.
  * @param propagation_step The time difference between x_{k+1}, x_{k} in
  * predict
  */
  TrackingVectorEstimator(TrackingVectorEstimatorConfig config,
                          std::chrono::duration<double> propagation_step);
  /**
  * @brief Perform a single correction using the provided
  * measurements
  *
  * @param marker_direction Measured marker direction in global frame
  */
  void correct(tf::Vector3 marker_direction,
               std::chrono::time_point<std::chrono::high_resolution_clock>
                   marker_time_stamp);
  /**
  * @brief Initialize the state of the filter by setting the marker direction
  * and noise levels to initial state stdev from config.
  *
  * @param marker_direction Marker direction to reset to
  */
  void initializeState(tf::Vector3 marker_direction);
  /**
  * @brief Helper function to move the internal filter state forward by dt
  *
  * @param velocity The control used to predict the marker direction at next
  * step
  */
  void predict(tf::Vector3 velocity);
  /**
  * @brief Get the estimated marker direction by the filter
  *
  * @return estimated marker direction in global frame
  */
  tf::Vector3 getMarkerDirection() {
    return tf::Vector3(filter_.statePost.at<double>(0),
                       filter_.statePost.at<double>(1),
                       filter_.statePost.at<double>(2));
  }
  /**
  * @brief The noise level in estimating marker direction
  *
  * @return the sqrt(diag(Covariance_marker_direction))
  */
  tf::Vector3 getMarkerNoise() {
    return tf::Vector3(sqrt(filter_.errorCovPost.at<double>(0, 0)),
                       sqrt(filter_.errorCovPost.at<double>(1, 1)),
                       sqrt(filter_.errorCovPost.at<double>(2, 2)));
  }
  /**
  * @brief Helper function to get the predicted (pre-correction) marker
  * direction from filter
  *
  * Note: This function does not call predict on the filter.
  *
  * @return marker direction before correction
  */
  tf::Vector3 getPredictedMarkerDirection() {
    return tf::Vector3(filter_.statePre.at<double>(0),
                       filter_.statePre.at<double>(1),
                       filter_.statePre.at<double>(2));
  }

  /**
  * @brief Set the initial state initialized flag to false
  * i.e the filter will reset the initial state using the
  * next available measurement instead of predicting
  * and correcting
  */
  void resetState() { initial_state_initialized_ = false; }

  /**
  * @brief Set the measurement covariance to the specified matrix
  *
  * @param covariance_mat the matrix to set to
  * @param marker_time_stamp the time stamp when the marker message has been
  * received
  */
  void setMeasurementCovariance(
      cv::Mat &covariance_mat,
      std::chrono::time_point<std::chrono::high_resolution_clock>
          marker_time_stamp);
};
