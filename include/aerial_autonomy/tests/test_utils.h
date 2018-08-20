#pragma once
#include <boost/filesystem.hpp>
#include <chrono>
#include <fstream>
#include <functional>
#include <gtest/gtest.h>
#include <parsernode/common.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <thread>

#include "aerial_autonomy/types/position_yaw.h"
#include "airm_mpc_controller_config.pb.h"
#include "uav_system_config.pb.h"

/**
 * @brief Namespace for helperfunctions that reduce code duplication
 * during testing
 */
namespace test_utils {

/**
 * @brief Check if data provided is the same the one stored in the
 * file
 *
 * @tparam T The type of element stored by the log
 * @param log vector data specifying the data to be stored
 * @param path The file path where data is compared to
 * @param delimiter the delimiter used in the file
 */
template <typename T>
void verifyFileData(std::vector<std::vector<T>> log,
                    boost::filesystem::path path, std::string delimiter) {
  std::fstream data_file(path.string(), std::fstream::in);
  ASSERT_TRUE(data_file.is_open());

  std::string data_point;
  for (auto data : log) {
    ASSERT_TRUE(std::getline(data_file, data_point));

    size_t next = 0;
    size_t last = 0;
    int i = -1;
    while ((next = data_point.find(delimiter, last)) != std::string::npos) {
      // Skip timestamp
      if (i >= 0) {
        ASSERT_EQ(data[i], std::stod(data_point.substr(last, next - last)));
      }
      last = next + 1;
      i++;
    }
    ASSERT_EQ(data[i], std::stod(data_point.substr(last)));
  }
  ASSERT_FALSE(std::getline(data_file, data_point));
}

/**
 * @brief Specialized version of verifyFileData for string data being stored
 *
 * @param log vector data specifying the data to be stored
 * @param path The file path where data is compared to
 * @param delimiter the delimiter used in the file
 */
template <>
void verifyFileData<std::string>(std::vector<std::vector<std::string>> log,
                                 boost::filesystem::path path,
                                 std::string delimiter) {
  std::fstream data_file(path.string(), std::fstream::in);
  ASSERT_TRUE(data_file.is_open());

  std::string data_point;
  for (auto data : log) {
    ASSERT_TRUE(std::getline(data_file, data_point));

    size_t next = 0;
    size_t last = 0;
    int i = -1;
    while ((next = data_point.find(delimiter, last)) != std::string::npos) {
      // Skip timestamp
      if (i >= 0) {
        ASSERT_EQ(data[i], data_point.substr(last, next - last));
      } else {
        ASSERT_EQ("#Time", data_point.substr(last, next - last));
      }
      last = next + 1;
      i++;
    }
    ASSERT_EQ(data[i], data_point.substr(last));
  }
  ASSERT_FALSE(std::getline(data_file, data_point));
}

/**
* @brief Assert that two tf vector3 are close to each other
*
* @param vec1 First vector
* @param vec2 second vector
* @param tol comparision tolerance
*/
template <class T>
void ASSERT_VEC_NEAR(const T vec1, const T &vec2, double tol = 1e-8) {
  ASSERT_NEAR(vec1.x(), vec2.x(), tol);
  ASSERT_NEAR(vec1.y(), vec2.y(), tol);
  ASSERT_NEAR(vec1.z(), vec2.z(), tol);
}

/**
 * @brief Assert that two tf::Transforms are close to each other
 * @param tf1 First transform
 * @param tf2 Second transform
 * @param tol Tolerance
 */
void ASSERT_TF_NEAR(const tf::Transform &tf1, const tf::Transform &tf2,
                    double tol = 1e-8) {
  ASSERT_VEC_NEAR(tf1.getOrigin(), tf2.getOrigin(), tol);
  ASSERT_NEAR((tf1.getRotation().inverse() * tf2.getRotation()).getAngle(), 0.,
              tol);
}

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
   * @param sleep_time Time to sleep between function calls
   *
   * @return  the output of input function at the end of the function evaluation
   */
  bool operator()(std::function<bool(void)> const &input_function,
                  std::chrono::duration<double> timeout,
                  std::chrono::duration<double> sleep_time =
                      std::chrono::milliseconds(100)) {
    auto start = std::chrono::system_clock::now();
    std::chrono::duration<double> duration(0);
    bool result;
    while ((result = input_function()) == !done &&
           duration.count() < timeout.count()) {
      std::this_thread::sleep_for(sleep_time);
      duration = std::chrono::system_clock::now() - start;
    }
    return result;
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
  std::string status_;                ///< string status received from UAV
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
* @brief Fill MPC config with default values
*
* @param config MPC Controller config
*/
void fillMPCConfig(AirmMPCControllerConfig &config) {
  auto lb = config.mutable_lower_bound_control();
  lb->Resize(6, 0);
  auto ub = config.mutable_upper_bound_control();
  ub->Resize(6, 0);
  config.set_lower_bound_control(0, 0.8);
  config.set_lower_bound_control(1, -0.3);
  config.set_lower_bound_control(2, -0.3);
  config.set_lower_bound_control(3, -0.3);
  config.set_lower_bound_control(4, -0.7);
  config.set_lower_bound_control(5, -0.7);

  config.set_upper_bound_control(0, 1.2);
  config.set_upper_bound_control(1, 0.3);
  config.set_upper_bound_control(2, 0.3);
  config.set_upper_bound_control(3, 0.3);
  config.set_upper_bound_control(4, 0.7);
  config.set_upper_bound_control(5, 0.7);

  config.set_goal_position_tolerance(0.2);
  config.set_goal_velocity_tolerance(0.25);
  config.set_goal_joint_angle_tolerance(0.2);
  config.set_goal_joint_velocity_tolerance(0.1);
  DDPMPCControllerConfig *ddp_config = config.mutable_ddp_config();
  ddp_config->set_n(100);
  ddp_config->set_max_cost(500);
  ddp_config->set_look_ahead_time(0.02);
  auto q = ddp_config->mutable_q();
  q->Resize(21, 0.0);
  auto qf = ddp_config->mutable_qf();
  qf->Resize(21, 0.1);
  auto r = ddp_config->mutable_r();
  r->Resize(6, 4.0);
  r->Set(0, 6.0); // Reduce thrust control
  r->Set(4, 1.0);
  r->Set(5, 1.0);
  for (int i = 0; i < 3; ++i) {
    // Pos cost
    ddp_config->set_qf(i, 100.0);
    ddp_config->set_q(i, 1.0);
    // Rpy cost
    ddp_config->set_qf(i + 3, 800.0);
    ddp_config->set_q(i + 3, 4.0);
    // Vel cost
    ddp_config->set_qf(i + 6, 100.0);
    ddp_config->set_q(i + 6, 4.0);
    // Rpydot cost
    ddp_config->set_qf(i + 9, 5.0);
    ddp_config->set_q(i + 9, 4.0);
  }
  for (int i = 0; i < 2; ++i) {
    // Joint angle cost
    ddp_config->set_qf(i + 15, 100.0);
    // Joint velocity cost
    ddp_config->set_qf(i + 17, 100.0);
  }
  ddp_config->set_debug(false);
  ddp_config->set_max_iters(5);
  config.set_weights_folder(
      "neural_network_model_data/tensorflow_model_vars_16_8_tanh/");
  config.set_use_code_generation(false);
}

/**
* @brief Fill MPC config into UAV arm system config part of UAV system config
*
* @param config UAV system config
*/
void fillMPCConfig(UAVSystemConfig &config) {
  auto mpc_config = config.mutable_uav_vision_system_config()
                        ->mutable_uav_arm_system_config()
                        ->mutable_mpc_controller_config();
  fillMPCConfig(*mpc_config);
}

/**
* @brief Create an MPC controller config with default values
*
* @return MPC controller config
*/
AirmMPCControllerConfig createMPCConfig() {
  AirmMPCControllerConfig config;
  fillMPCConfig(config);
  return config;
}

/**
* @brief create a default MPC Config
*
* @return config MPC Controller config
*/
void fillQuadMPCConfig(QuadMPCControllerConfig &config) {
  auto lb = config.mutable_lower_bound_control();
  lb->Resize(4, 0);
  auto ub = config.mutable_upper_bound_control();
  ub->Resize(4, 0);
  config.set_lower_bound_control(0, 0.4);
  config.set_lower_bound_control(1, -0.5);
  config.set_lower_bound_control(2, -0.5);
  config.set_lower_bound_control(3, -0.8);

  config.set_upper_bound_control(0, 1.4);
  config.set_upper_bound_control(1, 0.5);
  config.set_upper_bound_control(2, 0.5);
  config.set_upper_bound_control(3, 0.8);

  config.set_goal_position_tolerance(0.2);
  config.set_goal_velocity_tolerance(0.25);
  DDPMPCControllerConfig *ddp_config = config.mutable_ddp_config();
  ddp_config->set_n(100);
  ddp_config->set_max_cost(500);
  ddp_config->set_look_ahead_time(0.02);
  auto q = ddp_config->mutable_q();
  q->Resize(15, 0.0);
  auto qf = ddp_config->mutable_qf();
  qf->Resize(15, 0.1);
  auto r = ddp_config->mutable_r();
  r->Resize(4, 4.0);
  r->Set(0, 6.0); // Reduce thrust control
  for (int i = 0; i < 3; ++i) {
    // Pos cost
    ddp_config->set_qf(i, 50.0);
    ddp_config->set_q(i, 10.0);
    // Rpy cost
    ddp_config->set_qf(i + 3, 50.0);
    ddp_config->set_q(i + 3, 0.0);
    // Vel cost
    ddp_config->set_qf(i + 6, 50.0);
    ddp_config->set_q(i + 6, 10.0);
    // Rpydot cost
    ddp_config->set_qf(i + 9, 5.0);
    ddp_config->set_q(i + 9, 0.0);
  }
  ddp_config->set_debug(false);
  ddp_config->set_max_iters(2);
  config.set_use_code_generation(false);
}

/**
* @brief Create an MPC controller config with default values
*
* @return MPC controller config
*/
QuadMPCControllerConfig createQuadMPCConfig() {
  QuadMPCControllerConfig config;
  fillQuadMPCConfig(config);
  return config;
}

/**
 * @brief Convenience name to wait until input function returns true
 */
using waitUntilTrue = WaitUntilResult<true>;
/**
 * @brief Convenience name to wait until input function returns false
 */
using waitUntilFalse = WaitUntilResult<false>;
};
