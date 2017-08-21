#include "aerial_autonomy/controllers/relative_pose_controller.h"
#include <glog/logging.h>

bool RelativePoseController::runImplementation(
    std::tuple<tf::Transform, tf::Transform> sensor_data, tf::Transform goal,
    tf::Transform &control) {
  control = std::get<1>(sensor_data) * goal; //
  return true;
}

ControllerStatus RelativePoseController::isConvergedImplementation(
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

  const double &tolerance_pos = config_.goal_position_tolerance();
  if (abs_error_position.x() < tolerance_pos &&
      abs_error_position.y() < tolerance_pos &&
      abs_error_position.z() < tolerance_pos &&
      rot_diff < config_.goal_rotation_tolerance()) {
    VLOG_EVERY_N(1, 50) << "Reached goal";
    status.setStatus(ControllerStatus::Completed, "Reached goal");
  }
  return status;
}
