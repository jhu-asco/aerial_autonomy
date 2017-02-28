#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/String.h>

class EventPublisher {
public:
  EventPublisher(ros::NodeHandle &nh) : nh_(nh) {
    nh_.param<std::string>("event_name", event.data, "Land");
    event_pub_ = nh_.advertise<std_msgs::String>("event_manager", 1);
    event_timer_ = nh_.createTimer(ros::Duration(0.1),
                                   &EventPublisher::publishEvent, this);
  }

private:
  void publishEvent(const ros::TimerEvent &) { event_pub_.publish(event); }

  std_msgs::String event;
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
