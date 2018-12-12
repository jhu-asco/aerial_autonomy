#pragma once
#include "state_estimator_config.pb.h"
#include <Eigen/Dense>
#include <deque>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>
#include <vector>

class GenericStateEstimatorBackend {
public:
  GenericStateEstimatorBackend(StateEstimatorConfig config);
  // Methods to fill sensor data from front end
  using time_point =
      std::chrono::time_point<std::chrono::high_resolution_clock>;
  void addPositionData(time_point sensor_time,
                       const geometry_msgs::Vector3 position);
  void addOrientation(time_point sensor_time,
                      const geometry_msgs::Quaternion quat);
  void addBodyVelocity(time_point sensor_time,
                       const geometry_msgs::Vector3 body_velocity);
  void addOmega(time_point sensor_time, const geometry_msgs::Vector3 omega);
  void addAcc(time_point sensor_time,
              const geometry_msgs::Vector3 acceleration);
  void addAltitude(time_point sensor_time, const double altitude);
  void addTargetPosition(time_point sensor_time,
                         const geometry_msgs::Vector3 target_position);
  void addTargetOrientation(time_point sensor_time,
                            const geometry_msgs::Quaternion target_orientation);
  // Optimize based on available data
  void optimize();
  // Getters for accessing latest data
  tf::StampedTransform getPose();
  tf::StampedTransform getTargetPose();
  tf::Stamped<tf::Vector3> getBodyVelocity();
  tf::Stamped<tf::Vector3> getSpatialVelocity();
  tf::Stamped<tf::Vector3> getBodyAngularVelocity();

protected:
  template <class T> using DequeVector = std::deque<std::vector<T>>;
  DequeVector<geometry_msgs::Vector3> position_deque;
  DequeVector<geometry_msgs::Quaternion> orientation_deque;
  DequeVector<geometry_msgs::Vector3> velocity_deque;
  DequeVector<geometry_msgs::Vector3> angular_velocity_deque;
  DequeVector<geometry_msgs::Vector3> body_accelection_deque;
  DequeVector<double> altitude_deque;
  DequeVector<geometry_msgs::Vector3> target_position_deque;
  DequeVector<geometry_msgs::Quaternion> target_orientation_deque;
  DequeVector<Eigen::MatrixXd> prior_covariance;
  // Some member to store the output  of optimization (state knots)
};
