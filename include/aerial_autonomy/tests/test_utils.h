#pragma once
#include <chrono>
#include <functional>
#include <parsernode/common.h>
#include <tf/transform_datatypes.h>

namespace test_utils {
/**
 * @brief Functor class that waits until the input function results in the
 * template parameter
 * or timeout which ever occurs first
 *
 * @tparam done Specifies the result that the input function should provide when
 * the wait is over
 */
template <bool done> struct WaitUntilResult {
  /**
   * @brief Waits until timeout or the input function returns done
   *
   * @param input_function Logic function
   * @param timeout Timeout in seconds until to stop waiting for function output
   *
   * @return  the output of input function at the end of the function evaluation
   */
  bool operator()(std::function<bool(void)> const &input_function,
                  std::chrono::seconds timeout) {
    auto start = std::chrono::system_clock::now();
    std::chrono::seconds duration(0);
    while (input_function() == !done && duration.count() < timeout.count()) {
      ros::spinOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      duration = std::chrono::duration_cast<std::chrono::seconds>(
          std::chrono::system_clock::now() - start);
    }
    return input_function();
  }
};
/**
 * @brief Base Test fixture class for reducing code redundancy in system handler
 * tests
 */
struct BaseTestPubSubs {
  /**
   * @brief Constructor
   *
   * Initializes event, pose publishers, and system status subscriber
   */
  BaseTestPubSubs() : nh_send_("~common"), nh_receive_status_("~common") {
    event_pub_ = nh_send_.advertise<std_msgs::String>("event_manager", 1);
    pose_pub_ =
        nh_send_.advertise<geometry_msgs::PoseStamped>("goal_pose_command", 1);
    status_subscriber_ = nh_receive_status_.subscribe(
        "system_status", 1, &BaseTestPubSubs::statusCallback, this);
  }
  /**
   * @brief Helper function to publish specified event specified
   *
   * @param event Event to publish
   */
  void publishEvent(std::string event) {
    std_msgs::String event_msg;
    event_msg.data = event;
    event_pub_.publish(event_msg);
  }

  /**
   * @brief Helper function to publish specified position_yaw command
   *
   * @param position_yaw position and yaw to publish
   */
  void publishPoseCommand(PositionYaw position_yaw) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.x = position_yaw.x;
    pose_msg.pose.position.y = position_yaw.y;
    pose_msg.pose.position.z = position_yaw.z;
    pose_msg.pose.orientation =
        tf::createQuaternionMsgFromYaw(position_yaw.yaw);
    pose_pub_.publish(pose_msg);
  }

  /**
   * @brief Helper function to return position and yaw from UAV data
   *
   * @param quad_data UAV data returned by uav system
   *
   * @return Position and yaw of UAV
   */
  PositionYaw getPositionYaw(parsernode::common::quaddata quad_data) {
    return PositionYaw(quad_data.localpos.x, quad_data.localpos.y,
                       quad_data.localpos.z, quad_data.rpydata.z);
  }

  /**
   * @brief Helper function to check if status subscriber to connected UAV
   * status publisher
   *
   * @return true if connected
   */
  bool isStatusConnected() { return status_subscriber_.getNumPublishers() > 0; }

protected:
  std::string status_;
  ros::NodeHandle nh_send_;           ///< Send events
  ros::NodeHandle nh_receive_status_; ///< Receive status
  ros::Publisher event_pub_;          ///< Event publisher
  ros::Publisher pose_pub_;           ///< Pose command publisher
  ros::Subscriber status_subscriber_; ///< System status subscriber

  /**
   * @brief Callback to set the status received from UAV
   *
   * @param status Text status received from UAV
   */
  void statusCallback(std_msgs::String status) { status_ = status.data; }
};
/**
 * @brief Convenience name to wait until input function returns true
 */
using waitUntilTrue = WaitUntilResult<true>;
/**
 * @brief Convenience name to wait until input function returns false
 */
using waitUntilFalse = WaitUntilResult<false>;
};
