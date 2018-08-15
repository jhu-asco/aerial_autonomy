#include "aerial_autonomy/types/particle_reference_trajectory.h"
#include "aerial_autonomy/types/position_yaw.h"
#include <Eigen/Dense>

/**
 * @brief Reference trajectory for MPC quadrotor system
 */
class QuadParticleTrajectory
    : public ParticleTrajectory<Eigen::VectorXd, Eigen::VectorXd> {
public:
  /**
   * @brief Constructor
   *
   * @param goal_state  goal state in global frame
   * @param start_state start state in global frame
   * @param config reference config
   */
  QuadParticleTrajectory(PositionYaw goal_state, PositionYaw start_state,
                         ParticleReferenceConfig config);
  /**
   * @brief Gets the trajectory information at the specified time
   * @param t Time
   * @return Trajectory state and control
   */
  std::pair<Eigen::VectorXd, Eigen::VectorXd> atTime(double t) const;

  /**
   * @brief get goal at specified time
   *
   * Will give the end of reference trajectory
   *
   * @param double time
   *
   * @return State at end of reference trajectory
   */
  Eigen::VectorXd goal(double);

private:
  /**
   * @brief Find the intersection between linear portion and exponential portion
   *
   * @param gain          exponential gain
   * @param x_diff        The error between start and goal
   * @param max_velocity  The max velocity during the linear portion
   *
   * @return  The time until which linear portion is applied before switching to
   * exponential
   */
  double findTimeIntersection(double gain, double x_diff, double max_velocity);

private:
  static constexpr double gravity_magnitude_ = 9.81; ///< Gravity magnitude
  static constexpr double tol =
      1e-3; ///< Tolerance for checking intersection time is greater than 0
  Eigen::Vector4d
      time_intersection; ///< Switching times between linear, exponential
  Eigen::Vector4d linear_slopes; ///< Slope for linear portions
  PositionYaw error;             ///< Error between start and end state
};
