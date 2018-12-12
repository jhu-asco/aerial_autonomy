#pragma once
#include "generic_state_estimator_back_end.h"
#include "ros/ros.h"
#include "state_estimator_config.pb.h"
#include <geometry_msgs/Vector3.h>
#include <parsernode/parser.h>
#include <tf/tf.h>

class GenericStateEstimatorFrontend {
public:
  using time_point =
      std::chrono::time_point<std::chrono::high_resolution_clock>;
  GenericStateEstimatorFrontend(StateEstimatorConfig config,
                                parsernode::Parser &drone_hardware,
                                GenericStateEstimatorBackend &back_end);
  void trackingVectorCallback(time_point sensor_time, tf::Transform pose);
  void poseCallback(const geometry_msgs::TransformStamped &pose_input);
  void gyroCallback(time_point sensor_time,
                    const geometry_msgs::Vector3 &omega);
  void accCallback(time_point sensor_time, const geometry_msgs::Vector3 &acc);
  void altitudeCallback(time_point sensor_time, const double &altitude);
  void guidanceCallback(const geometry_msgs::Vector3Stamped &velocity);

private:
  StateEstimatorConfig config_;
  ros::NodeHandle nh_;
  ros::Subscriber pose_sub_;
  GenericStateEstimatorBackend &back_end_;
};
