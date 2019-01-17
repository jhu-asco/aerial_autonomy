#pragma once
#include "state_estimator_config.pb.h"
#include "state_estimator_status.h"
#include <Eigen/Dense>
#include <aerial_autonomy/common/chrono_stamped.h>
#include <deque>
#include <tf/tf.h>
#include <vector>

class StateEstimatorBackend {
public:
  StateEstimatorBackend(StateEstimatorConfig config);
  // Methods to fill sensor data from front end
  using time_point =
      std::chrono::time_point<std::chrono::high_resolution_clock>;
  void addPositionData(time_point sensor_time, const Eigen::Vector3d position);
  void addOrientation(time_point sensor_time, const tf::Quaternion quat);
  void addBodyVelocity(time_point sensor_time,
                       const Eigen::Vector3d body_velocity);
  void addOmega(time_point sensor_time, const Eigen::Vector3d omega);
  void addAcceleration(time_point sensor_time,
                       const Eigen::Vector3d acceleration);
  void addAltitude(time_point sensor_time, const double altitude);
  void addTargetPose(time_point sensor_time, const tf::Tranform target_pose);
  // Optimize based on available data
  StateEstimatorStatus::OptimizationStatus optimize();
  Eigen::VectorXd getCurrentState();

protected:
  template <class T> using DequeVector = std::deque<std::vector<T>>;
  DequeVector<Eigen::Vector3d> position_deque;
  DequeVector<tf::Quaternion> orientation_deque;
  DequeVector<Eigen::Vector3d> velocity_deque;
  DequeVector<Eigen::Vector3d> angular_velocity_deque;
  DequeVector<Eigen::Vector3d> body_accelection_deque;
  DequeVector<double> altitudde_deque;
  DequeVector<Eigen::Vector3d> target_position_deque;
  DequeVector<tf::Quaternion> target_orientation_deque;
  DequeVector<Eigen::MatrixXd> prior_covariance;
  Eigen::VectorXd state_knots;
};
