#pragma once
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/particle_state_yaw.h"
#include "aerial_autonomy/types/snap.h"
#include "aerial_autonomy/types/reference_trajectory.h"
#include "polynomial_reference_config.pb.h"
#include <Eigen/Dense>
#include "aerial_autonomy/types/polynomial_reference_trajectory.h"

/**
 * @brief Reference trajectory for MPC quadrotor system
 */
class AdaptivePolynomialReferenceTrajectory
    : public ReferenceTrajectory<ParticleStateYaw, Snap> {
public:
  /**
   * @brief Constructor
   *
   * @param goal_state  goal state in global frame
   * @param start_state start state in global frame
   * @param config reference config
   */
  AdaptivePolynomialReferenceTrajectory(PositionYaw goal_state, PositionYaw start_state,
                                PolynomialReferenceConfig config);

  /**
   * @brief Gets the trajectory information at the specified time
   * @param t Time
   * @return Trajectory state and control
   */
  std::pair<ParticleStateYaw, Snap> atTime(double t) const;

  /**
   * @brief get goal at specified time
   *
   * Will give the end of reference trajectory
   *
   * @param double time
   *
   * @return State at end of reference trajectory
   */
  ParticleStateYaw goal(double);

  /**
   * @brief Find the coefficient matrix for polynomial at specified time
   *
   * Finds coefficients up to snap
   *
   * @param t  Time
   * @param degree The degree of polynomial
   * @param dimensions of the constraints at t
   *
   * @return Matrix of dimension [5xdegree+1] corresponding to position,
   * velocity, acceleration, jerk and snap
   */
  Eigen::MatrixXd findBasisMatrix(double t, int degree, int dimensions) const;

  /**
   * @brief Get additive sinusoidal noise
   *
   * @param t time
   * @param a amplitude
   * @param nu frequency
   *
   * @return noise, noise_dot, noise_ddot
   */
  Eigen::Vector3d getNoise(double t, double a, double nu) const;

private:
  PolynomialReferenceTrajectory wrapped_traj_;
};
