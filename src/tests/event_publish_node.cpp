#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/String.h>

class EventPublisher {
public:
  EventPublisher(ros::NodeHandle &nh) : nh_(nh) {
    if (!nh_.getParam("event_name", event)) {
      event = "Land";
    }

    if (event == "PositionYaw") {
      event_pub_ =
          nh_.advertise<geometry_msgs::PoseStamped>("goal_pose_command", 1);
    } else {
      event_pub_ = nh_.advertise<std_msgs::String>("event_manager", 1);
    }
    event_timer_ = nh_.createTimer(ros::Duration(0.1),
                                   &EventPublisher::publishEvent, this);
  }

private:
  void publishEvent(const ros::TimerEvent &) {
    if (event == "PositionYaw") {
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = 1;
      pose.pose.position.y = 2;
      pose.pose.position.z = 3;
      pose.pose.orientation.w = 1;
      pose.pose.orientation.x = 0;
      pose.pose.orientation.y = 0;
      pose.pose.orientation.z = 0;
      event_pub_.publish(pose);
    } else {
      std_msgs::String event_message;
      event_message.data = event;
      event_pub_.publish(event_message);
    }
  }

  std::string event;
  ros::Timer event_timer_;
  ros::Publisher event_pub_;
  ros::NodeHandle &nh_;
};

void sigintHandler(int sig) { ros::shutdown(); }

int main(int argc, char **argv) {
  ros::init(argc, argv, "event_publish_node",
            ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  EventPublisher event_publisher(nh);

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, sigintHandler);

  ros::spin();
  return 0;
}
