#include "aerial_autonomy/controllers/rpyt_based_relative_pose_controller.h"
#include "aerial_autonomy/log/log.h"

#include <glog/logging.h>

bool RPYTBasedRelativePoseController::runImplementation(
    std::tuple<tf::Transform, tf::Transform, VelocityYawRate> sensor_data,
    PositionYaw goal, RollPitchYawRateThrust &control) {
  bool result = true;
  VelocityYawRate desired_velocity_yawrate;
  tf::Transform current_transform = std::get<0>(sensor_data);
  auto transform_tuple =
      std::make_tuple(current_transform, std::get<1>(sensor_data));
  velocity_based_relative_pose_controller_.setGoal(goal);
  result &= velocity_based_relative_pose_controller_.run(
      transform_tuple, desired_velocity_yawrate);
  double current_yaw = tf::getYaw(current_transform.getRotation());
  auto velocity_yawrate_yaw_tuple =
      std::make_tuple(std::get<2>(sensor_data), current_yaw);
  rpyt_based_velocity_controller_.setGoal(desired_velocity_yawrate);
  result &=
      rpyt_based_velocity_controller_.run(velocity_yawrate_yaw_tuple, control);
  return result;
}

ControllerStatus RPYTBasedRelativePoseController::isConvergedImplementation(
    std::tuple<tf::Transform, tf::Transform, VelocityYawRate> sensor_data,
    PositionYaw goal) {
  // TODO use status from rpyt_based velocity controller also
  auto transform_tuple =
      std::make_tuple(std::get<0>(sensor_data), std::get<1>(sensor_data));
  return velocity_based_relative_pose_controller_.isConverged(transform_tuple);
}
