#pragma once
#include <aerial_autonomy/trackers/simple_tracker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Transform.h>
#include <ros/ros.h>

/**
 * @brief ROS tracker that takes in a tracked transform
 */
class SimulatedROSTracker : public SimpleTracker {
public:
  /**
   * @brief Constructor
   *
   * @param drone_hardware Quad hardware
   * @param camera_transform Camera transform
   * @param nh_namespace Namespace for ros nodehandle
   */
  SimulatedROSTracker(parsernode::Parser &drone_hardware,
                      tf::Transform camera_transform,
                      std::string nh_namespace = "");

private:
  /**
   * @brief set tracked object transform
   *
   * @param transform_msg transform message
   */
  void targetTransformCallback(const geometry_msgs::Transform &transform_msg);
  /**
   * @brief set tracked object position
   *
   * @param point_msg desired position
   */
  void targetPointCallback(const geometry_msgs::Point &point_msg);

private:
  ros::NodeHandle nh_;                   ///< Node Handle
  ros::Subscriber target_transform_sub_; ///< Transform subscriber
  ros::Subscriber target_point_sub_;     ///< Point subscriber
};
