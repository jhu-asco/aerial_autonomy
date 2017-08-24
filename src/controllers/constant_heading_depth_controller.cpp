#include "aerial_autonomy/controllers/constant_heading_depth_controller.h"
#include "aerial_autonomy/common/math.h"
#include <glog/logging.h>

bool ConstantHeadingDepthController::runImplementation(
    PositionYaw sensor_data, Position goal, VelocityYawRate &control) {
  tf::Vector3 current_tracking_vector(sensor_data.x, sensor_data.y,
                                      sensor_data.z);
  tf::Vector3 desired_tracking_vector(goal.x, goal.y, goal.z);
  tf::Vector3 desired_tracking_direction =
      desired_tracking_vector / desired_tracking_vector.length();
  tf::Vector3 tracking_error =
      (current_tracking_vector - desired_tracking_vector);
  tf::Vector3 tracking_error_radial =
      desired_tracking_direction *
      (tracking_error.dot(desired_tracking_direction));
  tf::Vector3 tracking_error_tangential =
      tracking_error - tracking_error_radial;
  tf::Vector3 desired_vel_tf =
      tracking_error_radial * config_.radial_gain() +
      tracking_error_tangential * config_.tangential_gain();

  if (desired_vel_tf.length() > config_.max_velocity()) {
    desired_vel_tf *= config_.max_velocity() / desired_vel_tf.length();
  } else if (desired_vel_tf.length() < config_.min_velocity()) {
    desired_vel_tf *= config_.min_velocity() / desired_vel_tf.length();
  }

  double error_yaw =
      math::angleWrap(std::atan2(desired_tracking_direction.getY(),
                                 desired_tracking_direction.getX()) -
                      sensor_data.yaw);
  double yaw_rate =
      math::clamp(config_.yaw_gain() * error_yaw, -config_.max_yaw_rate(),
                  config_.max_yaw_rate());
  control = VelocityYawRate(desired_vel_tf.getX(), desired_vel_tf.getY(),
                            desired_vel_tf.getZ(), yaw_rate);
  return true;
}

ControllerStatus ConstantHeadingDepthController::isConvergedImplementation(
    PositionYaw sensor_data, Position goal) {
  double error_yaw =
      math::angleWrap(std::atan2(goal.y, goal.x) - sensor_data.yaw);
  Position error = Position(sensor_data.x, sensor_data.y, sensor_data.z) - goal;
  ControllerStatus status(ControllerStatus::Active);
  status << " Error Position, Yaw: " << error.x << error.y << error.z
         << error_yaw;
  const PositionControllerConfig &position_controller_config =
      config_.position_controller_config();
  const config::Position &tolerance_pos =
      position_controller_config.goal_position_tolerance();
  const double &tolerance_yaw = position_controller_config.goal_yaw_tolerance();
  // Compare
  if (std::abs(error.x) < tolerance_pos.x() &&
      std::abs(error.y) < tolerance_pos.y() &&
      std::abs(error.z) < tolerance_pos.z() &&
      std::abs(error_yaw) < tolerance_yaw) {
    VLOG_EVERY_N(1, 50) << "Reached goal";
    status.setStatus(ControllerStatus::Completed, "Reached Goal");
  }
  return status;
}
