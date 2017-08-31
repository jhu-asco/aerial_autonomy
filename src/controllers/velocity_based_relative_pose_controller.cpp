#include "aerial_autonomy/controllers/velocity_based_relative_pose_controller.h"
#include "aerial_autonomy/log/log.h"

#include <glog/logging.h>

bool VelocityBasedRelativePoseController::runImplementation(
    std::tuple<tf::Transform, tf::Transform> sensor_data, tf::Transform goal,
    VelocityYaw &control) {
  tf::Transform current_pose = std::get<0>(sensor_data) * goal;
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
  tf::Transform tracked_pose = std::get<1>(sensor_data);

  tf::Transform relative_pose = tracked_pose.inverse() * current_pose;
  tf::Vector3 relative_position = relative_pose.getOrigin();
  tf::Vector3 goal_position = goal.getOrigin();
  tf::Quaternion relative_quat = relative_pose.getRotation();
  tf::Quaternion goal_quat = goal.getRotation();
  double rot_diff = goal_quat.angleShortestPath(relative_quat);

  tf::Vector3 abs_error_position =
      (relative_position - goal_position).absolute();
  ControllerStatus status(ControllerStatus::Active);
  status << "Error Position, Rotation: " << abs_error_position.x()
         << abs_error_position.y() << abs_error_position.z() << rot_diff;
  const tf::Quaternion &current_rot = current_pose.getRotation();
  const tf::Vector3 &current_trans = current_pose.getOrigin();
  const tf::Quaternion &tracked_rot = tracked_pose.getRotation();
  const tf::Vector3 &tracked_trans = tracked_pose.getOrigin();
  DATA_LOG("velocity_based_relative_pose_controller")
      << abs_error_position.x() << abs_error_position.y()
      << abs_error_position.z() << rot_diff << current_trans.x()
      << current_trans.y() << current_trans.z() << current_rot.w()
      << current_rot.x() << current_rot.y() << current_rot.z()
      << tracked_trans.x() << tracked_trans.y() << tracked_trans.z()
      << tracked_rot.w() << tracked_rot.x() << tracked_rot.y()
      << tracked_rot.z() << DataStream::endl;

  const config::Position &tolerance_pos = config_.goal_position_tolerance();
  if (abs_error_position.x() < tolerance_pos.x() &&
      abs_error_position.y() < tolerance_pos.y() &&
      abs_error_position.z() < tolerance_pos.z() &&
      rot_diff < config_.goal_rotation_tolerance()) {
    VLOG_EVERY_N(1, 50) << "Reached goal";
    status.setStatus(ControllerStatus::Completed, "Reached goal");
  }
  return status;
}
