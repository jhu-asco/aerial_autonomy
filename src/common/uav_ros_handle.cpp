#include "aerial_autonomy/common/uav_ros_handle.h"
#include "aerial_autonomy/common/math.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>


#include "rpyt_based_position_controller_config.pb.h"

UAVRosHandle::UAVRosHandle(UAVSystem &uav_system)
    : nh_("~uav_state"), uav_system_(uav_system),
      ref_controller_config_sub_(nh_.subscribe("ref_controller_config", 1,
          &UAVRosHandle::refControllerConfigCallback, this)),
      pose_pub_(nh_.advertise<geometry_msgs::PoseStamped>("pose", 1)),
      twist_pub_(nh_.advertise<geometry_msgs::TwistStamped>("twist", 1)) {}

void UAVRosHandle::publish() {
  auto data = uav_system_.getUAVData();

  publishPose(data);
  publishTwist(data);
}

void UAVRosHandle::refControllerConfigCallback(std_msgs::Float32MultiArray message) {
  auto config = uav_system_.getReferenceControllerConfig();
  auto velocity_config = config.mutable_rpyt_based_velocity_controller_config();
  auto velocity_position_config =
      config.mutable_velocity_based_position_controller_config();

  velocity_position_config->set_position_gain(message.data[0]);
  velocity_position_config->set_z_gain(message.data[1]);
  velocity_config->set_kp_xy(message.data[2]);
  velocity_config->set_kp_z(message.data[3]);

  uav_system_.setReferenceControllerConfig(config); 
}

void UAVRosHandle::publishPose(const parsernode::common::quaddata &data) {
  auto q = math::rpyToQuat(data.rpydata.x, data.rpydata.y, data.rpydata.z);

  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "world";

  pose.pose.position.x = data.localpos.x;
  pose.pose.position.y = data.localpos.y;
  pose.pose.position.z = data.localpos.z;

  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();

  pose_pub_.publish(pose);
}

void UAVRosHandle::publishTwist(const parsernode::common::quaddata &data) {
  geometry_msgs::TwistStamped twist;
  twist.header.stamp = ros::Time::now();
  twist.header.frame_id = "world";

  twist.twist.linear.x = data.linvel.x;
  twist.twist.linear.y = data.linvel.y;
  twist.twist.linear.z = data.linvel.z;

  twist.twist.angular.x = data.omega.x;
  twist.twist.angular.y = data.omega.y;
  twist.twist.angular.z = data.omega.z;

  twist_pub_.publish(twist);
}
