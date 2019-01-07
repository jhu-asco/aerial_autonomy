#pragma once
#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/common/file_utils.h"
#include "aerial_autonomy/common/math.h"
#include "aerial_autonomy/types/particle_state.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/reference_trajectory.h"
#include "aerial_autonomy/types/snap.h"

#include <Eigen/Dense>

/**
* @brief A circular trajectory for testing.
* Currently adds a
*/
class CircleReferenceTrajectory
    : public ReferenceTrajectory<ParticleState, Snap> {
public:
  /**
  * @brief Constructor
  *
  */
  CircleReferenceTrajectory() {}

  /**
  * @brief Constructor
  *
  * @param py Goal from connector.
  * initializes the radius to the py radius, z to the py z, and w to 1.
  */
  CircleReferenceTrajectory(const PositionYaw &py) {
    radius_ = sqrt(py.x * py.x + py.y * py.y);
    z_ = py.z;
    w_ = 1.0;
    // Z oscillation constants
    z_amp_ = 0.1;
    z_freq_ = 1.0;
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
    double theta = w_ * t;
    double x = radius_ * cos(theta);
    double y = radius_ * sin(theta);
    Position p(x, y, z_ + z_amp_ * sin(z_freq_ * t));
    Velocity v(-w_ * y, w_ * x, z_amp_ * z_freq_ * cos(z_freq_ * t));
    Acceleration a(-w_ * w_ * x, -w_ * w_ * y,
                   -z_amp_ * z_freq_ * z_freq_ * sin(z_freq_ * t));
    Jerk j(w_ * w_ * w_ * y, -w_ * w_ * w_ * x,
           -z_amp_ * z_freq_ * z_freq_ * z_freq_ * cos(z_freq_ * t));
    Snap snap(w_ * w_ * w_ * w_ * x, w_ * w_ * w_ * w_ * y,
              z_amp_ * z_freq_ * z_freq_ * z_freq_ * z_freq_ *
                  sin(z_freq_ * t));

    return std::pair<ParticleState, Snap>(ParticleState(p, v, a, j), snap);
  }

private:
  double radius_, z_, w_, z_amp_, z_freq_;
};
