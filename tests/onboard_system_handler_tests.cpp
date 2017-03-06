#include <aerial_autonomy/basic_events.h>
#include <aerial_autonomy/onboard_system_handler.h>
#include <aerial_autonomy/state_machines/basic_state_machine.h>
#include <aerial_autonomy/tests/sample_parser.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

using namespace basic_events;

class OnboardSystemHandlerTests : public ::testing::Test {
public:
  OnboardSystemHandlerTests()
      : nh_(), nh_send_(), sample_parser_(new SampleParser()),
        onboard_system_handler_(nh_, sample_parser_) {
    sample_parser_->setBatteryPercent(100);
    event_pub_ = nh_send_.advertise<std_msgs::String>("event_manager", 1);
    pose_pub_ =
        nh_send_.advertise<geometry_msgs::PoseStamped>("goal_pose_command", 1);
    ros::spinOnce();
  }

  void publishEvent(std::string event) {
    std_msgs::String event_msg;
    event_msg.data = event;
    event_pub_.publish(event_msg);
    // wait for threads to process
    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    ros::spinOnce();
  }

  void publishPoseCommand(PositionYaw pose) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.x = pose.x;
    pose_msg.pose.position.y = pose.y;
    pose_msg.pose.position.z = pose.z;
    pose_msg.pose.orientation.w = 1; // TODO(matt): use yaw
    pose_pub_.publish(pose_msg);
    // wait for threads to process
    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    ros::spinOnce();
  }

  parsernode::common::quaddata getQuadData() {
    parsernode::common::quaddata quad_data;
    sample_parser_->getquaddata(quad_data);
    return quad_data;
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_send_;

  ros::Publisher event_pub_;
  ros::Publisher pose_pub_;

public:
  SampleParser *sample_parser_;
  OnboardSystemHandler<LogicStateMachine, BasicEventManager<LogicStateMachine>>
      onboard_system_handler_;
};

TEST_F(OnboardSystemHandlerTests, Constructor) {}

TEST_F(OnboardSystemHandlerTests, TestConnections) {
  while (!onboard_system_handler_.isConnected()) {
  }
  SUCCEED();
}

TEST_F(OnboardSystemHandlerTests, ProcessEvents) {
  while (!onboard_system_handler_.isConnected()) {
  }
  // Check takeoff works
  publishEvent("Takeoff");
  ASSERT_EQ(getQuadData().quadstate, "takeoff");
  // Check subsequent event works
  publishEvent("Land");
  ASSERT_EQ(getQuadData().quadstate, "land");
  // Check pose command works
  publishEvent("Takeoff");
  ASSERT_EQ(getQuadData().quadstate, "takeoff");
  sample_parser_->setaltitude(2.0);
  PositionYaw pose_command(1, 2, 3, 0);
  publishPoseCommand(pose_command);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  ASSERT_EQ(PositionYaw(getQuadData().localpos.x, getQuadData().localpos.y,
                        getQuadData().localpos.z, getQuadData().rpydata.z),
            pose_command);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "onboard_system_handler_tests");
  return RUN_ALL_TESTS();
}
