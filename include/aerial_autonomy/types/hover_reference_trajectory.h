#pragma once
#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/common/file_utils.h"
#include "aerial_autonomy/common/math.h"
#include "aerial_autonomy/types/particle_state.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/reference_trajectory.h"
#include "aerial_autonomy/types/snap.h"
#include "minimum_snap_reference_trajectory_config.pb.h"

#include <Eigen/Dense>

/**
* @brief A minimum snap trajectory containing controls, states and timestamps
* Gets state/control information using unconstrained QP
*/
class HoverReferenceTrajectory
    : public ReferenceTrajectory<ParticleState, Snap> {
public:
  /**
  * @brief Constructor
  *
  * @param der_order_in Order of the derivative subject to optimization
  * @param tau_vec_in Array of time intervals
  * @param path_in nby3 matrix containing waypoints (x,y,z)
  *
  */
  HoverReferenceTrajectory() {}

  /**
  * @brief Constructor
  *
  * @param ref_config Reference trajectory config
  *
  */
  HoverReferenceTrajectory(const PositionYaw &py) {
    x = py.x;
    y = py.y;
    z = py.z;
  }

  /**
  * @brief Gets the trajectory information at the specified time using minimum
  * snap
  *
  * @param t Time
  *
  * @return Trajectory state and control
  */
  std::pair<ParticleState, Snap> atTime(double t) const {
    Position p(x, y, z);
    Velocity v(0, 0, 0);
    Acceleration a(0, 0, 0);
    Jerk j(0, 0, 0);
    Snap snap(0, 0, 0);

    return std::pair<ParticleState, Snap>(ParticleState(p, v, a, j), snap);
  }

private:
  double x, y, z;
};
