#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <signal.h>

class PosePublisher {
public:
  PosePublisher(ros::NodeHandle &nh) : nh_(nh) {
    pose.pose.position.x = 1;
    pose.pose.position.y = 2;
    pose.pose.position.z = 3;
    pose.pose.orientation.w = 1;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;

    pose_pub_ =
        nh_.advertise<geometry_msgs::PoseStamped>("goal_pose_command", 1);
    pose_timer_ =
        nh_.createTimer(ros::Duration(0.1), &PosePublisher::publishPose, this);
  }

private:
  void publishPose(const ros::TimerEvent &) { pose_pub_.publish(pose); }

  geometry_msgs::PoseStamped pose;
  ros::Timer pose_timer_;
  ros::Publisher pose_pub_;
  ros::NodeHandle &nh_;
};

void sigintHandler(int sig) { ros::shutdown(); }

int main(int argc, char **argv) {
  ros::init(argc, argv, "pose_publish_node",
            ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  PosePublisher pose_publisher(nh);

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, sigintHandler);

  ros::spin();
  return 0;
}
