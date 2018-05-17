#pragma once

#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/qrotor_bs_control.h"
#include "aerial_autonomy/types/qrotor_bs_state.h"
#include "aerial_autonomy/types/reference_trajectory.h"
#include "qrotor_backstepping_controller_config.pb.h"

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

  std::tuple<ParticleState, Snap>
  getGoalFromReference(const ReferenceTrajectory<ParticleState, Snap> &);
  /**
  * @brief Controller config
  */
  QrotorBacksteppingControllerConfig config_;
};
