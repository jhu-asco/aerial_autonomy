#pragma once
#include "ros/ros.h"
#include "state_estimator_backend.h"
#include "state_estimator_config.pb.h"
#include <geometry_msgs/Vector3.h>
#include <parsernode/parser.h>
#include <tf/tf.h>

class StateEstimatorFrontend {
public:
  StateEstimatorFrontend(StateEstimatorConfig config,
                         parsernode::Parser &drone_hardware,
                         StateEstimatorBackend &backend);
  void poseCallback(const geometry_msgs::TransformStamped &pose_input);
  void guidanceCallback(const geometry_msgs::Vector3Stamped &velocity);
  void gyroCallback(const geometry_msgs::Vector3 &omega);
  void accCallback(const geometry_msgs::Vector3 &acc);
  void altitudeCallback(const double &altitude);
  void trackingVectorCallback(tf::Transform pose);
  CumulativeSensorStatus getStatus() { return sensor_status_; }

private:
  StateEstimatorConfig config_;
  ros::NodeHandle nh_;
  ros::Subscriber pose_sub_;
  StateEstimatorBackend &backend_;
  CumulativeSensorStatus sensor_status_;
};
