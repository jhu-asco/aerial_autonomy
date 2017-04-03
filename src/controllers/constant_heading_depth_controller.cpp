#include "aerial_autonomy/controllers/constant_heading_depth_controller.h"
#include "aerial_autonomy/common/math.h"
#include <tf/tf.h>

VelocityYawRate
ConstantHeadingDepthController::runImplementation(PositionYaw sensor_data,
                                                  Position goal) {
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
  }

  double error_yaw = std::atan2(desired_tracking_direction.getY(),
                                desired_tracking_direction.getX()) -
                     sensor_data.yaw;
  double yaw_rate =
      math::clamp(config_.yaw_gain() * math::angleWrap(error_yaw),
                  -config_.max_yaw_rate(), config_.max_yaw_rate());
  return VelocityYawRate(desired_vel_tf.getX(), desired_vel_tf.getY(),
                         desired_vel_tf.getZ(), yaw_rate);
}
