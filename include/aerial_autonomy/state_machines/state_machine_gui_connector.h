#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf_conversions/tf_eigen.h>

#include <aerial_autonomy/types/position_yaw.h>

/**
* @brief Connects a logic state machine to the GUI over a ROS interface
*
* @tparam EventManagerT event manager type used to trigger events to state
* machine
* @tparam LogicStateMachineT logic state machine type used to trigger pose
* events
*/
template <class EventManagerT, class LogicStateMachineT>
class StateMachineGUIConnector {
public:
  /**
  * @brief Constructor
  * Creates subscriber to event callbacks to connect GUI to state machine
  *
  * @param nh NodeHandle to create subscribers
  * @param event_manager Event manager to trigger events based on name alone
  * @param logic_state_machine State machine to trigger events directly
  */
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
  /**
  * @brief Callback to trigger events based on name
  *
  * @param event_data name of event to trigger
  */
  void eventCallback(const std_msgs::StringConstPtr &event_data) {
    VLOG(2) << "Triggering event: " << event_data;
    event_manager_.triggerEvent(event_data->data, logic_state_machine_);
  }

  /**
  * @brief Callback to trigger event for pose command
  *
  * @param pose command to send to state machine
  */
  void poseCommandCallback(const geometry_msgs::PoseStamped &pose) {
    PositionYaw pose_command;
    pose_command.x = pose.pose.position.x;
    pose_command.y = pose.pose.position.y;
    pose_command.z = pose.pose.position.z;
    pose_command.yaw = tf::getYaw(pose.pose.orientation);
    VLOG(2) << "Received Pose command: " << pose_command.x << "\t"
            << pose_command.y << "\t" << pose_command.z << "\t"
            << pose_command.yaw;
    logic_state_machine_.process_event(pose_command);
  }

  /**
  * @brief pose command subscriber
  */
  ros::Subscriber pose_command_sub_;
  /**
  * @brief event command subscriber
  */
  ros::Subscriber event_manager_sub_;
  /**
  * @brief Event manager to trigger event by name
  */
  EventManagerT &event_manager_;
  /**
  * @brief Logic state machine to trigger specific events
  */
  LogicStateMachineT &logic_state_machine_;
};
