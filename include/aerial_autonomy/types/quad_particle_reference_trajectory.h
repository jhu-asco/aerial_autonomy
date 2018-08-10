#include "aerial_autonomy/types/particle_reference_trajectory.h"
#include "aerial_autonomy/types/position_yaw.h"
#include <Eigen/Dense>

class QuadParticleTrajectory
    : public ParticleTrajectory<Eigen::VectorXd, Eigen::VectorXd> {
public:
  QuadParticleTrajectory(PositionYaw goal_state, PositionYaw current_state,
                         ParticleReferenceConfig config);
  /**
* @brief Gets the trajectory information at the specified time
* @param t Time
* @return Trajectory state and control
*/
  std::pair<Eigen::VectorXd, Eigen::VectorXd> atTime(double t) const;

  double findTimeIntersection(double gain, double x_diff, double max_velocity);

  Eigen::VectorXd goal(double);

private:
  static constexpr double gravity_magnitude_ = 9.81; ///< Gravity magnitude
  static constexpr double tol = 1e-3;
  Eigen::Vector4d time_intersection;
  Eigen::Vector4d linear_slopes;
  PositionYaw error;
  std::pair<Eigen::VectorXd, Eigen::VectorXd> atTimeLinearPortion(double t);
};
