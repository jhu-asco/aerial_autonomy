#include "aerial_autonomy/trackers/simulated_ros_tracker.h"
#include <tf/tf.h>

SimulatedROSTracker::SimulatedROSTracker(parsernode::Parser &drone_hardware,
                                         tf::Transform camera_transform,
                                         std::string nh_namespace)
    : SimpleTracker(drone_hardware, camera_transform), nh_(nh_namespace),
      target_transform_sub_(
          nh_.subscribe("target_tracker_transform", 1,
                        &SimulatedROSTracker::targetTransformCallback, this)),
      target_point_sub_(nh_.subscribe("target_tracker_point", 1,
                                      &SimulatedROSTracker::targetPointCallback,
                                      this)) {}

void SimulatedROSTracker::targetTransformCallback(
    const geometry_msgs::Transform &transform_msg) {
  tf::Transform target_transform;
  tf::transformMsgToTF(transform_msg, target_transform);
  setTargetPoseGlobalFrame(target_transform);
}

void SimulatedROSTracker::targetPointCallback(
    const geometry_msgs::Point &point_msg) {
  ROS_INFO("Setting target position: %f, %f, %f", point_msg.x, point_msg.y,
           point_msg.z);
  setTargetPositionGlobalFrame(Position(point_msg.x, point_msg.y, point_msg.z));
}
