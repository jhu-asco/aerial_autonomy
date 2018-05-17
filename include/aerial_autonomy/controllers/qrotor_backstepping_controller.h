#pragma once

#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/particle_state.h"
#include "aerial_autonomy/types/qrotor_bs_control.h"
#include "aerial_autonomy/types/qrotor_bs_state.h"
#include "aerial_autonomy/types/reference_trajectory.h"
#include "aerial_autonomy/types/snap.h"
#include "qrotor_backstepping_controller_config.pb.h"

#include <Eigen/Dense>

/**
 * @brief A trajectory-tracking backstepping controller for a quadrotor.
 * Based on <Kobilarov, "Trajectory tracking of a class of underactuated systems
 * with
 * external disturbances", American Control Conference, 2013>
 */
class QrotorBacksteppingController
    : public Controller<QrotorBSState, std::tuple<ParticleState, Snap>,
                        QrotorBSControl> {
public:
  /**
  * @brief Constructor
  * @param config Controller config
  */
  QrotorBacksteppingController(QrotorBacksteppingControllerConfig config)
      : config_(config) {
    // TODO compute P from Q
    P_ = Eigen::MatrixXd::Identity(6, 6);
  }

protected:
  /**
   * @brief Run the control loop.  Uses a backstepping controller to track the
   * desired trajectory.
   * @param sensor_data Current qrotor state
   * @param goal Reference trajectory
   * @param control Controls
   * @return true if command to reach goal is found
   */
  bool runImplementation(QrotorBSState sensor_data,
                         ReferenceTrajectory<ParticleState, Snap> goal,
                         QrotorBSControl &control);
  /**
  * @brief Check if controller has converged (reached the end of the reference)
  *
  * @param sensor_data Current qrotor state
  * @param goal Reference trajectory
  *
  * @return Controller status
  */
  ControllerStatus
  isConvergedImplementation(QrotorBSState sensor_data,
                            ReferenceTrajectory<ParticleState, Snap> goal);

  /**
  * @brief Gets current goal and derivatives from the reference
  * @param ref Reference trajectory
  * @return Current goal
  */
  std::tuple<ParticleState, Snap>
  getGoalFromReference(const ReferenceTrajectory<ParticleState, Snap> &ref);
  /**
  * @brief Controller config
  */
  QrotorBacksteppingControllerConfig config_;

private:
  Eigen::Matrix<double, 6, 6> P_;
};
