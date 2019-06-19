#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include "aerial_autonomy/common/math.h"
#include "aerial_autonomy/common/uav_state_publisher.h"

UAVStatePublisher::UAVStatePublisher(UAVSystem &uav_system)
    : nh_("~uav_state"), uav_system_(uav_system),
      pose_pub_(nh_.advertise<geometry_msgs::PoseStamped>("pose", 1)),
      twist_pub_(nh_.advertise<geometry_msgs::TwistStamped>("twist", 1)) {}

void UAVStatePublisher::publish() {
  auto data = uav_system_.getUAVData();

  publishPose(data);
  publishTwist(data);
}

void UAVStatePublisher::publishPose(const parsernode::common::quaddata &data) {
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

void UAVStatePublisher::publishTwist(const parsernode::common::quaddata &data) {
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
