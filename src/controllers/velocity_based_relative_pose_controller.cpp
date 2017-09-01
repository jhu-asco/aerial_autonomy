#include "aerial_autonomy/controllers/velocity_based_relative_pose_controller.h"
#include "aerial_autonomy/log/log.h"

#include <glog/logging.h>

bool VelocityBasedRelativePoseController::runImplementation(
    std::tuple<tf::Transform, tf::Transform> sensor_data, tf::Transform goal,
    VelocityYaw &control) {
  tf::Transform current_pose = std::get<0>(sensor_data);
  tf::Transform desired_pose = std::get<1>(sensor_data) * goal;

  double current_roll, current_pitch, current_yaw;
  current_pose.getBasis().getRPY(current_roll, current_pitch, current_yaw);

  double desired_roll, desired_pitch, desired_yaw;
  desired_pose.getBasis().getRPY(desired_roll, desired_pitch, desired_yaw);

  PositionYaw desired_position_yaw(
      desired_pose.getOrigin().getX(), desired_pose.getOrigin().getY(),
      desired_pose.getOrigin().getZ(), desired_yaw);
  PositionYaw current_position_yaw(
      current_pose.getOrigin().getX(), current_pose.getOrigin().getY(),
      current_pose.getOrigin().getZ(), current_yaw);
  position_controller_.setGoal(desired_position_yaw);

  return position_controller_.run(current_position_yaw, control);
}

ControllerStatus VelocityBasedRelativePoseController::isConvergedImplementation(
    std::tuple<tf::Transform, tf::Transform> sensor_data, tf::Transform goal) {
  tf::Transform current_pose = std::get<0>(sensor_data);
  tf::Transform desired_pose = std::get<1>(sensor_data) * goal;

  double current_roll, current_pitch, current_yaw;
  current_pose.getBasis().getRPY(current_roll, current_pitch, current_yaw);

  double desired_roll, desired_pitch, desired_yaw;
  desired_pose.getBasis().getRPY(desired_roll, desired_pitch, desired_yaw);

  PositionYaw desired_position_yaw(
      desired_pose.getOrigin().getX(), desired_pose.getOrigin().getY(),
      desired_pose.getOrigin().getZ(), desired_yaw);
  PositionYaw current_position_yaw(
      current_pose.getOrigin().getX(), current_pose.getOrigin().getY(),
      current_pose.getOrigin().getZ(), current_yaw);
  position_controller_.setGoal(desired_position_yaw);

  return position_controller_.isConverged(current_position_yaw);
}
