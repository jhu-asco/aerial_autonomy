#include "aerial_autonomy/controllers/rpyt_based_reference_controller.h"

using namespace Eigen;

std::pair<VelocityYawRate, PositionYaw>
RPYTBasedReferenceControllerEigen::getReference(const VectorXd &state) {
  PositionYaw position_yaw(state(0), state(1), state(2), state(5));
  VelocityYawRate velocity_yawrate(state(6), state(7), state(8), state(11));
  return std::make_pair(velocity_yawrate, position_yaw);
}
