#include "aerial_autonomy/controllers/quad_polynomial_reference_controller.h"
#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/types/polynomial_reference_trajectory.h"
#include <glog/logging.h>
#include <tf/tf.h>

QuadPolynomialReferenceController::QuadPolynomialReferenceController(
    PolynomialReferenceConfig config)
    : config_(config) {}

bool QuadPolynomialReferenceController::runImplementation(
    std::pair<PositionYaw, tf::Transform> sensor_data, PositionYaw goal,
    ReferenceTrajectoryPtr<Eigen::VectorXd, Eigen::VectorXd> &control) {
  tf::Transform goal_tf;
  conversions::positionYawToTf(goal, goal_tf);
  tf::Transform desired_pose = sensor_data.second * goal_tf;
  double desired_yaw = tf::getYaw(desired_pose.getRotation());
  PositionYaw desired_pose_yaw(desired_pose.getOrigin().getX(),
                               desired_pose.getOrigin().getY(),
                               desired_pose.getOrigin().getZ(), desired_yaw);
  control.reset(new PolynomialReferenceTrajectory(desired_pose_yaw,
                                                  sensor_data.first, config_));
  return true;
}

void QuadPolynomialReferenceController::useNoise(bool flag) {
  LOG(INFO) << "Use noise: " << flag;
  config_.set_add_noise(flag);
}

void QuadPolynomialReferenceController::reset() {
  LOG(INFO) << "Resetting poly reference";
  useNoise(false); // reset any noise flags in config
}
