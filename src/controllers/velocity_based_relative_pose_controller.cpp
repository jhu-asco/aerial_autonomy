#include "aerial_autonomy/controllers/velocity_based_relative_pose_controller.h"
#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/log/log.h"

#include <glog/logging.h>

bool VelocityBasedRelativePoseController::runImplementation(
    std::tuple<tf::Transform, tf::Transform> sensor_data, PositionYaw goal,
    VelocityYawRate &control) {
  tf::Transform goal_tf;
  conversions::positionYawToTf(goal, goal_tf);
  tf::Transform current_pose = std::get<0>(sensor_data);
  tf::Transform desired_pose = std::get<1>(sensor_data) * goal_tf;

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
  position_controller_.setGoal(desired_position_yaw, false);

  auto status = position_controller_.run(current_position_yaw, control);
  DATA_LOG("velocity_based_relative_pose_controller")
      << desired_pose.getOrigin().getX() << desired_pose.getOrigin().getY()
      << desired_pose.getOrigin().getZ() << desired_yaw
      << current_pose.getOrigin().getX() << current_pose.getOrigin().getY()
      << current_pose.getOrigin().getZ() << current_yaw << control.x
      << control.y << control.z << control.yaw_rate << DataStream::endl;
  return status;
}

ControllerStatus VelocityBasedRelativePoseController::isConvergedImplementation(
    std::tuple<tf::Transform, tf::Transform> sensor_data, PositionYaw goal) {
  tf::Transform goal_tf;
  conversions::positionYawToTf(goal, goal_tf);
  tf::Transform current_pose = std::get<0>(sensor_data);
  tf::Transform tracked_pose = std::get<1>(sensor_data);
  tf::Transform desired_pose = tracked_pose * goal_tf;

  double current_roll, current_pitch, current_yaw;
  current_pose.getBasis().getRPY(current_roll, current_pitch, current_yaw);

  double desired_roll, desired_pitch, desired_yaw;
  desired_pose.getBasis().getRPY(desired_roll, desired_pitch, desired_yaw);

  tf::Transform frame_relative_pose = tracked_pose.inverse() * current_pose;
  tf::Vector3 frame_relative_error =
      frame_relative_pose.getOrigin() - goal_tf.getOrigin();
  double frame_relative_roll, frame_relative_pitch, frame_relative_yaw;
  frame_relative_pose.getBasis().getRPY(
      frame_relative_roll, frame_relative_pitch, frame_relative_yaw);
  double frame_relative_yaw_error =
      math::angleWrap(frame_relative_yaw - goal.yaw);

  PositionYaw desired_position_yaw(
      desired_pose.getOrigin().getX(), desired_pose.getOrigin().getY(),
      desired_pose.getOrigin().getZ(), desired_yaw);
  PositionYaw current_position_yaw(
      current_pose.getOrigin().getX(), current_pose.getOrigin().getY(),
      current_pose.getOrigin().getZ(), current_yaw);
  position_controller_.setGoal(desired_position_yaw, false);

  ControllerStatus status(
      position_controller_.isConverged(current_position_yaw).status());
  status << "Error Position, Yaw: " << frame_relative_error.x()
         << frame_relative_error.y() << frame_relative_error.z()
         << frame_relative_yaw_error;
  return status;
}

aerial_autonomy::VelocityBasedPositionControllerDynamicConfig
VelocityBasedRelativePoseController::getDefaultConfig() const {
  return position_controller_.getDefaultConfig();
}

void VelocityBasedRelativePoseController::updateConfig(
    const aerial_autonomy::VelocityBasedPositionControllerDynamicConfig
        &dynamic_config) {
  position_controller_.updateConfig(dynamic_config);
}
