#include <aerial_autonomy/state_machines/pick_place_state_machine.h>
#include <aerial_autonomy/system_handlers/uav_arm_system_handler.h>
#include <aerial_autonomy/pick_place_events.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

using namespace uav_basic_events;

class UAVArmSystemHandlerTests : public ::testing::Test {
public:
  UAVArmSystemHandlerTests() : nh_(), nh_send_(), nh_receive_status_() {
    // Configure system
    UAVSystemHandlerConfig uav_system_handler_config;
    uav_system_handler_config.set_uav_parser_type(
        "quad_simulator_parser/QuadSimParser");
    uav_system_handler_config.mutable_uav_arm_system_handler_config()->set_arm_parser_type("ArmSimulator");
    uav_system_handler_config.mutable_uav_system_config()
        ->set_minimum_takeoff_height(0.4);
    for (int i = 0; i < 6; ++i) {
      uav_system_handler_config.mutable_uav_system_config()
          ->mutable_uav_vision_system_config()
          ->add_camera_transform(0.0);

      uav_system_handler_config.mutable_uav_system_config()
          ->mutable_uav_vision_system_config()->mutable_uav_arm_system_config()
          ->add_arm_transform(0.0);

      uav_system_handler_config.mutable_uav_system_config()
          ->mutable_uav_vision_system_config()->mutable_uav_arm_system_config()
          ->add_arm_goal_transform(0.0);
    }

    uav_system_handler_.reset(
        new UAVArmSystemHandler<
            PickPlaceStateMachine,
            pick_place_events::PickPlaceEventManager<
                PickPlaceStateMachine>>(nh_, uav_system_handler_config));
    event_pub_ = nh_send_.advertise<std_msgs::String>("event_manager", 1);
    pose_pub_ =
        nh_send_.advertise<geometry_msgs::PoseStamped>("goal_pose_command", 1);
    status_subscriber_ = nh_receive_status_.subscribe(
        "system_status", 1, &UAVArmSystemHandlerTests::statusCallback, this);
    //uav_system_handler_->powerArm(true);
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
  std::unique_ptr<
      UAVArmSystemHandler<PickPlaceStateMachine,
                             pick_place_events::PickPlaceEventManager<
                                 PickPlaceStateMachine>>>
      uav_system_handler_; ///< system contains robot system, state machine
};

TEST_F(UAVArmSystemHandlerTests, Constructor) {}

TEST_F(UAVArmSystemHandlerTests, TestConnections) {
  while (!uav_system_handler_->isConnected()) {
  }
  SUCCEED();
}

TEST_F(UAVArmSystemHandlerTests, ProcessEvents) {
  while (!uav_system_handler_->isConnected()) {
  }
  // Check takeoff works
  publishEvent("Takeoff");
  // Wait for internal events to process
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  ASSERT_TRUE(uav_system_handler_->getUAVData().armed);
  ASSERT_EQ(uav_system_handler_->getUAVData().localpos.z, 0.5);
  // Check subsequent event works
  publishEvent("Land");
  // Wait for internal events to process
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  ASSERT_FALSE(uav_system_handler_->getUAVData().armed);
  ASSERT_EQ(uav_system_handler_->getUAVData().localpos.z, 0.0);
}

TEST_F(UAVArmSystemHandlerTests, ProcessPoseCommand) {
  while (!uav_system_handler_->isConnected()) {
  }
  // Check pose command works
  publishEvent("Takeoff");
  // Wait for internal events to process
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
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

TEST_F(UAVArmSystemHandlerTests, ReceiveStatus) {
  while (!isStatusConnected())
    ;
  for (int count = 0; count < 2; ++count) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ros::spinOnce(); // To receive status data
  }
  ASSERT_FALSE(status_.empty());
}

/// \todo Matt Add visual servoing tests

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "uav_arm_system_handler_tests");
  return RUN_ALL_TESTS();
}
