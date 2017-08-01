#include <aerial_autonomy/state_machines/uav_state_machine.h>
#include <aerial_autonomy/system_handlers/uav_system_handler.h>
#include <aerial_autonomy/uav_basic_events.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

using namespace uav_basic_events;

class UAVSystemHandlerTests : public ::testing::Test {
public:
  UAVSystemHandlerTests() : nh_(), nh_send_(), nh_receive_status_() {
    // Configure system
    UAVSystemHandlerConfig uav_system_handler_config;
    uav_system_handler_config.set_uav_parser_type(
        "quad_simulator_parser/QuadSimParser");
    uav_system_handler_config.mutable_uav_system_config()
        ->set_minimum_takeoff_height(0.4);

    uav_system_handler_.reset(
        new UAVSystemHandler<UAVStateMachine, UAVEventManager<UAVStateMachine>>(
            nh_, uav_system_handler_config));
    event_pub_ = nh_send_.advertise<std_msgs::String>("event_manager", 1);
    pose_pub_ =
        nh_send_.advertise<geometry_msgs::PoseStamped>("goal_pose_command", 1);
    status_subscriber_ = nh_receive_status_.subscribe(
        "system_status", 1, &UAVSystemHandlerTests::statusCallback, this);
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
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  bool isStatusConnected() { return status_subscriber_.getNumPublishers() > 0; }

protected:
  std::string status_;
  void statusCallback(std_msgs::String status) { status_ = status.data; }

private:
  ros::NodeHandle nh_;      ///< NodeHandle used by onboard nodehandler
  ros::NodeHandle nh_send_; ///< Send events
  ros::NodeHandle nh_receive_status_; ///< Receive status

  ros::Publisher event_pub_;          ///< Event publisher
  ros::Publisher pose_pub_;           ///< Pose command publisher
  ros::Subscriber status_subscriber_; ///< System status subscriber

public:
  std::unique_ptr<UAVSystemHandler<UAVStateMachine,
                                   UAVEventManager<UAVStateMachine>>>
      uav_system_handler_; ///< system contains robot system, state machine
};

TEST_F(UAVSystemHandlerTests, Constructor) {}

TEST_F(UAVSystemHandlerTests, TestConnections) {
  while (!uav_system_handler_->isConnected()) {
  }
  SUCCEED();
}

TEST_F(UAVSystemHandlerTests, ProcessEvents) {
  while (!uav_system_handler_->isConnected()) {
  }
  // Check takeoff works
  publishEvent("Takeoff");
  ASSERT_TRUE(uav_system_handler_->getUAVData().armed);
  ASSERT_EQ(uav_system_handler_->getUAVData().localpos.z, 0.5);
  // Check subsequent event works
  publishEvent("Land");
  ASSERT_FALSE(uav_system_handler_->getUAVData().armed);
  ASSERT_EQ(uav_system_handler_->getUAVData().localpos.z, 0.0);
}

TEST_F(UAVSystemHandlerTests, ProcessPoseCommand) {
  while (!uav_system_handler_->isConnected()) {
  }
  // Check pose command works
  publishEvent("Takeoff");
  ASSERT_TRUE(uav_system_handler_->getUAVData().armed);
  ASSERT_EQ(uav_system_handler_->getUAVData().localpos.z, 0.5);
  PositionYaw pose_command(1, 2, 3, 0);
  publishPoseCommand(pose_command);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  parsernode::common::quaddata quad_data = uav_system_handler_->getUAVData();
  ASSERT_EQ(PositionYaw(quad_data.localpos.x, quad_data.localpos.y,
                        quad_data.localpos.z, quad_data.rpydata.z),
            pose_command);
}

TEST_F(UAVSystemHandlerTests, ReceiveStatus) {
  while (!isStatusConnected())
    ;
  for (int count = 0; count < 2; ++count) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ros::spinOnce(); // To receive status data
  }
  ASSERT_FALSE(status_.empty());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "uav_system_handler_tests");
  return RUN_ALL_TESTS();
}
