#include "aerial_autonomy/controllers/rpyt_based_reference_controller.h"

using namespace Eigen;

std::pair<VelocityYawRate, PositionYaw>
RPYTBasedReferenceControllerEigen::getReference(const VectorXd &state) {
  PositionYaw position_yaw(state(0), state(1), state(2), state(5));
  VelocityYawRate velocity_yawrate(state(6), state(7), state(8), state(11));
  return std::make_pair(velocity_yawrate, position_yaw);
}

Eigen::Vector3d RPYTBasedReferenceControllerEigen::getAcceleration(
    const std::pair<VectorXd, VectorXd> &state_control_pair) {
  const auto &state = state_control_pair.first;
  const auto &control = state_control_pair.second;
  tf::Transform transform;
  conversions::transformRPYToTf(state(3), state(4), state(5), transform);
  tf::Vector3 acc = transform.getBasis().getColumn(2) * control(0) * gravity_;
  return Eigen::Vector3d(acc[0], acc[1], acc[2]);
}
