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
class CircleReferenceTrajectory
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
  CircleReferenceTrajectory() {}

  /**
  * @brief Constructor
  *
  * @param ref_config Reference trajectory config
  *
  */ 
  CircleReferenceTrajectory(const PositionYaw &py){
    radius = sqrt(py.x*py.x + py.y*py.y);
    z = py.z;
    w = 1.0;
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

    // Type conversion
    double theta = w*t;
    double x = radius*cos(theta);
    double y = radius*sin(theta);
    Position p(x, y, z + 0.1*sin(10*theta));
    Velocity v(-w*y, w*x, w*cos(10*theta));
    Acceleration a(-w*w*x, -w*w*y, -10*w*w*sin(theta));
    Jerk j(w*w*w*y, -w*w*w*x, -100*w*w*w*cos(theta));
    Snap snap(w*w*w*w*x, w*w*w*w*y, 1000*w*w*w*w*sin(theta));

    return std::pair<ParticleState, Snap>(ParticleState(p, v, a, j), snap);
  }

private:
  double radius,z,w;
};
