#include "aerial_autonomy/controllers/quad_polynomial_adaptive_reference_controller.h"
#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/types/adaptive_polynomial_reference_trajectory.h"
#include <glog/logging.h>
#include <tf/tf.h>

QuadPolynomialAdaptiveReferenceController::QuadPolynomialReferenceController(
    PolynomialReferenceConfig config)
    : config_(config) {}

bool QuadPolynomialAdaptiveReferenceController::runImplementation(
    std::pair<PositionYaw, tf::Transform> sensor_data, PositionYaw goal,
    ReferenceTrajectoryPtr<ParticleStateYaw, Snap> &control) {
  tf::Transform goal_tf;
  conversions::positionYawToTf(goal, goal_tf);
  tf::Transform desired_pose = sensor_data.second * goal_tf;
  double desired_yaw = tf::getYaw(desired_pose.getRotation());
  PositionYaw desired_pose_yaw(desired_pose.getOrigin().getX(),
                               desired_pose.getOrigin().getY(),
                               desired_pose.getOrigin().getZ(), desired_yaw);
  control.reset(new AdaptivePolynomialReferenceTrajectory(desired_pose_yaw,
                                                  sensor_data.first, config_));
  return true;
}

void QuadPolynomialAdaptiveReferenceController::useNoise(bool flag) {
  LOG(INFO) << "Use noise: " << flag;
  config_.set_add_noise(flag);
}

void QuadPolynomialAdaptiveReferenceController::reset() {
  LOG(INFO) << "Resetting poly reference";
  useNoise(false); // reset any noise flags in config
}
