#include "aerial_autonomy/types/particle_reference_trajectory.h"
#include "aerial_autonomy/types/position_yaw.h"
#include <Eigen/Dense>

class QuadParticleTrajectory
    : public ParticleTrajectory<Eigen::VectorXd, Eigen::VectorXd> {
public:
  QuadParticleTrajectory(PositionYaw goal_state, PositionYaw current_state,
                         ParticleReferenceConfig config)
      : ParticleTrajectory<Eigen::VectorXd, Eigen::VectorXd>(
            goal_state, current_state, config) {}
  /**
* @brief Gets the trajectory information at the specified time
* @param t Time
* @return Trajectory state and control
*/
  virtual std::pair<Eigen::VectorXd, Eigen::VectorXd> atTime(double t) const;

private:
  static constexpr double gravity_magnitude_ = 9.81; ///< Gravity magnitude
};
