#pragma once

#include "aerial_autonomy/common/math.h"
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
    : public Controller<std::tuple<double, QrotorBSState>,
                        std::tuple<ParticleState, Snap>, QrotorBSControl> {
public:
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Matrix63d = Eigen::Matrix<double, 6, 3>;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;
  using Matrix36d = Eigen::Matrix<double, 3, 6>;

  /**
  * @brief Constructor
  * @param config Controller config
  */
  QrotorBacksteppingController(QrotorBacksteppingControllerConfig config)
      : config_(config), m_(config_.mass()), e_(0, 0, 1),
        ag_(0, 0, -config_.acc_gravity()) {
    // Compute P from Q, K
    Eigen::Vector3d kp(config_.kp_xy(), config_.kp_xy(), config_.kp_z());
    Eigen::Vector3d kd(config_.kd_xy(), config_.kd_xy(), config_.kd_z());
    K_.leftCols<3>() = kp.asDiagonal();
    K_.rightCols<3>() = kd.asDiagonal();

    A_ = Eigen::MatrixXd::Zero(6, 6);
    A_.topRightCorner<3, 3>() = Eigen::MatrixXd::Identity(3, 3);

    B_ = Eigen::MatrixXd::Zero(6, 3);
    B_.bottomLeftCorner<3, 3>() = (1. / m_) * Eigen::MatrixXd::Identity(3, 3);

    J_ = Eigen::Vector3d(config_.jx(), config_.jy(), config_.jz()).asDiagonal();

    Vector6d Qvec;
    Qvec << config_.qx(), config_.qy(), config_.qz(), config_.qvx(),
        config_.qvy(), config_.qvz();
    Q_ = Qvec.asDiagonal();

    auto AmBK = A_ - B_ * K_;
    P_ = math::sylvester(AmBK.transpose(), AmBK, Q_);
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
  bool runImplementation(std::tuple<double, QrotorBSState> sensor_data,
                         ReferenceTrajectory<ParticleState, Snap> goal,
                         QrotorBSControl &control);
  /**
  * @brief Check if controller has converged (reached the end of the reference)
  *
  * @param sensor_data Current qrotor state and time relative to trajectory
  * origin
  * @param goal Reference trajectory
  *
  * @return Controller status
  */
  ControllerStatus
  isConvergedImplementation(std::tuple<double, QrotorBSState> sensor_data,
                            ReferenceTrajectory<ParticleState, Snap> goal);

  /**
  * @brief Gets current goal and derivatives from the reference
  * @param ref Reference trajectory
  * @return Current goal
  */
  std::tuple<ParticleState, Snap>
  getGoalFromReference(double current_time,
                       const ReferenceTrajectory<ParticleState, Snap> &ref);
  /**
  * @brief Controller config
  */
  QrotorBacksteppingControllerConfig config_;

private:
  Matrix6d A_;
  Matrix6d P_;
  Matrix36d K_;
  Matrix63d B_;
  double m_;
  Eigen::Vector3d e_; // thrust vector in body-frame
  Eigen::Vector3d ag_;
  Eigen::Matrix3d J_;
  Matrix6d Q_;
};
