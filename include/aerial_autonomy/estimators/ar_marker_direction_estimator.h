#pragma once
#include "ar_marker_direction_estimator_config.pb.h"
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
class ARMarkerDirectionEstimator {

private:
  ARMarkerDirectionEstimatorConfig config_;
  cv::KalmanFilter filter_;
  const double zero_tolerance_;
  bool initial_state_initialized_;
  template <class T> void setCovarianceMatrix(cv::Mat &matrix, T stdev_vector) {
    matrix = cv::Mat_<double>::zeros(6, 6);
    CHECK_EQ(stdev_vector.size(), 6)
        << "Process noise vector should be of size 6";
    for (int i = 0; i < 6; ++i) {
      CHECK_GE(stdev_vector[i], zero_tolerance_)
          << "Stdev should be atleast greater than tolerance";
      matrix.at<double>(i, i) = (stdev_vector[i] * stdev_vector[i]);
    }
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
  ARMarkerDirectionEstimator(ARMarkerDirectionEstimatorConfig config,
                             std::chrono::duration<double> propagation_step);
  /**
  * @brief Perform a single prediction and a correction using the provided
  * measurements
  *
  * @param marker_direction Measured marker direction in global frame
  * @param velocity Measured velocity in global frame
  */
  void estimate(tf::Vector3 marker_direction, tf::Vector3 velocity);
  /**
  * @brief Initialize the state of the filter by setting the marker direction,
  * velocity
  * and noise levels to initial state stdev from config.
  *
  * @param marker_direction Marker direction to reset to
  * @param velocity Velocity to reset to
  */
  void initializeState(tf::Vector3 marker_direction, tf::Vector3 velocity);
  /**
  * @brief Helper function to move the internal filter state forward by dt
  */
  void predict() { filter_.predict(); }
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
  * @brief Get the estimated velocity by the filter
  *
  * @return estimated velocity in global frame
  */
  tf::Vector3 getVelocity() {
    return tf::Vector3(filter_.statePost.at<double>(3),
                       filter_.statePost.at<double>(4),
                       filter_.statePost.at<double>(5));
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
  * @brief The noise level in estimating velocity vector
  *
  * @return the sqrt(diag(Covariance_velocity_vector))
  */
  tf::Vector3 getVelocityNoise() {
    return tf::Vector3(sqrt(filter_.errorCovPost.at<double>(3, 3)),
                       sqrt(filter_.errorCovPost.at<double>(4, 4)),
                       sqrt(filter_.errorCovPost.at<double>(5, 5)));
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
  * @brief Helper function to get predicted velocity (pre-correction) from
  * filter
  *
  * Note: This function does not call predict on the filter.
  *
  * @return velocity vector before correction
  */
  tf::Vector3 getPredictedVelocity() {
    return tf::Vector3(filter_.statePre.at<double>(3),
                       filter_.statePre.at<double>(4),
                       filter_.statePre.at<double>(5));
  }

  /**
  * @brief Set the initial state initialized flag to false
  * i.e the filter will reset the initial state using the
  * next available measurement instead of predicting
  * and correcting
  */
  void resetState() { initial_state_initialized_ = false; }
};
