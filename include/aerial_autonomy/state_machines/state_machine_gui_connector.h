#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf_conversions/tf_eigen.h>

#include <aerial_autonomy/types/position_yaw.h>

/**
 * @brief Connects a logic state machine to the GUI over a ROS interface
 */
template <class EventManagerT, class LogicStateMachineT>
class StateMachineGUIConnector {
public:
  StateMachineGUIConnector(ros::NodeHandle &nh, EventManagerT &event_manager,
                           LogicStateMachineT &logic_state_machine)
      : event_manager_(event_manager),
        logic_state_machine_(logic_state_machine) {
    event_manager_sub_ = nh.subscribe(
        "event_manager", 1, &StateMachineGUIConnector::eventCallback, this);
    pose_command_sub_ =
        nh.subscribe("goal_pose_command", 1,
                     &StateMachineGUIConnector::poseCommandCallback, this);
  }

  /**
   * @brief Returns whether the GUI connector is connected to an event publisher
   * @return Returns true if connected
   */
  bool isEventManagerConnected() {
    std::cout << "check" << std::endl;
    return event_manager_sub_.getNumPublishers() > 0;
  }

  /**
   * @brief Returns whether the GUI connector is connected to an pose command
   * publisher
   * @return Returns true if connected
   */
  bool isPoseCommandConnected() {
    return pose_command_sub_.getNumPublishers() > 0;
  }

private:
  void eventCallback(const std_msgs::StringConstPtr &event_data) {
    std::cout << "got message" << std::endl;
    event_manager_.triggerEvent(event_data->data, logic_state_machine_);
  }

  void poseCommandCallback(const geometry_msgs::PoseStamped &pose) {
    PositionYaw pose_command;
    pose_command.x = pose.pose.position.x;
    pose_command.y = pose.pose.position.y;
    pose_command.z = pose.pose.position.z;
    pose_command.yaw = tf::getYaw(pose.pose.orientation);
    logic_state_machine_.process_event(pose_command);
  }

  ros::Subscriber pose_command_sub_;
  ros::Subscriber event_manager_sub_;
  EventManagerT &event_manager_;
  LogicStateMachineT &logic_state_machine_;
};
