#include "aerial_autonomy/controllers/ddp_quad_mpc_controller.h"
#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/log/log.h"
#include <gcop/quad_casadi_system.h>

constexpr int DDPQuadMPCController::state_size_,
    DDPQuadMPCController::control_size_;

void DDPQuadMPCController::loadQuadParameters(Eigen::Vector3d &kp_rpy,
                                              Eigen::Vector3d &kd_rpy,
                                              Eigen::VectorXd &p,
                                              QuadMPCControllerConfig &config) {
  kp_rpy << config.kp_roll(), config.kp_pitch(), 0.01;
  kd_rpy << config.kd_roll(), config.kd_pitch(), config.kd_yaw();
  p << config.default_thrust_gain();
}

DDPQuadMPCController::DDPQuadMPCController(
    QuadMPCControllerConfig config,
    std::chrono::duration<double> controller_duration)
    : DDPCasadiMPCController(config.ddp_config(), controller_duration),
      config_(config) {
  // Instantiate system
  Eigen::Vector3d kp_rpy, kd_rpy;
  loadQuadParameters(kp_rpy, kd_rpy, kt_, config);
  /// \todo Add defaults if vector size is wrong
  lb_ = conversions::vectorProtoToEigen(config.lower_bound_control());
  ub_ = conversions::vectorProtoToEigen(config.upper_bound_control());
  if (lb_.size() != control_size_ || ub_.size() != control_size_) {
    LOG(WARNING) << "Lower/Upper bound size incorrect: " << lb_.size() << ", "
                 << ub_.size();
    controller_config_status_ = false;
    return;
  }
  std::cout << "Lb: " << lb_.transpose() << std::endl;
  std::cout << "Ub: " << ub_.transpose() << std::endl;
  sys_.reset(new gcop::QuadCasadiSystem(kt_, kp_rpy, kd_rpy, lb_, ub_,
                                        config.use_code_generation()));
  VLOG(1) << "Instantiating Step function";
  sys_->instantiateStepFunction();
  // cost
  auto ddp_config = config.ddp_config();
  unsigned int N = ddp_config.n();
  double h = ddp_config.h();
  double tf = h * N;
  VLOG(1) << "Manifold size: " << (sys_->X.n);
  xf_.resize(state_size_);
  xf_.setZero();
  cost_.reset(new gcop::LqCost<Eigen::VectorXd>(*sys_, tf, xf_));
  cost_->SetReference(&xds_, &uds_);
  VLOG(1) << "Created cost function";
  if (ddp_config.q_size() != sys_->X.n || ddp_config.qf_size() != sys_->X.n ||
      ddp_config.r_size() != sys_->U.n) {
    LOG(WARNING) << "Cost config sizes do not match";
    controller_config_status_ = false;
    return;
  }
  cost_->Q =
      (conversions::vectorProtoToEigen(*ddp_config.mutable_q())).asDiagonal();
  cost_->Qf =
      (conversions::vectorProtoToEigen(*ddp_config.mutable_qf())).asDiagonal();
  cost_->R =
      (conversions::vectorProtoToEigen(*ddp_config.mutable_r())).asDiagonal();
  cost_->UpdateGains();
  // References:
  VLOG(1) << "Trajectory length: " << N;
  // states
  Eigen::VectorXd default_state(state_size_);
  default_state.setZero();
  xs_.resize(N + 1, default_state);
  // Controls
  resetControls(); // Set controls to default values and resets DDP
  // Copy reference from states, controls
  xds_ = xs_;
  uds_ = us_;
  VLOG(1) << "Done setting up ddp";
  // Setting up logger
  DATA_HEADER("ddp_quad_mpc_controller") << "Errorx"
                                         << "Errory"
                                         << "Errorz"
                                         << "Errorvx"
                                         << "Errorvy"
                                         << "Errorvz"
                                         << "thrust_d"
                                         << "rd"
                                         << "pd"
                                         << "yaw_rate_d"
                                         << "J"
                                         << "x_ref"
                                         << "y_ref"
                                         << "z_ref"
                                         << "r_ref"
                                         << "p_ref"
                                         << "y_ref"
                                         << "vx_ref"
                                         << "vy_ref"
                                         << "vz_ref"
                                         << "Loop timer" << DataStream::endl;
}

DDPQuadMPCController::ControlType DDPQuadMPCController::stationaryControl() {
  Eigen::VectorXd ui(control_size_);
  ui << 1.0, 0, 0, 0;
  return ui;
}

ControllerStatus DDPQuadMPCController::isConvergedImplementation(
    MPCInputs<StateType> sensor_data, GoalType goal) {
  if (!controller_config_status_) {
    LOG(WARNING) << "Controller config invalid!";
    return ControllerStatus(ControllerStatus::Critical);
  }
  boost::mutex::scoped_lock lock(copy_mutex_);
  ControllerStatus controller_status = ControllerStatus::Active;
  double t0 = sensor_data.time_since_goal;
  Eigen::VectorXd end_goal = goal->goal(t0);
  Eigen::Vector3d error_position =
      sensor_data.initial_state.segment<3>(0) - end_goal.segment<3>(0);
  Eigen::Vector3d error_velocity =
      sensor_data.initial_state.segment<3>(6) - end_goal.segment<3>(6);
  double error_yaw =
      math::angleWrap(sensor_data.initial_state(5) - end_goal(5));
  if (error_position.squaredNorm() < config_.goal_position_tolerance() *
                                         config_.goal_position_tolerance() &&
      error_velocity.squaredNorm() < config_.goal_velocity_tolerance() *
                                         config_.goal_velocity_tolerance() &&
      std::abs(error_yaw) < config_.goal_yaw_tolerance()) {
    VLOG(1) << "Controller Converged!";
    controller_status.setStatus(ControllerStatus::Completed,
                                "Converged to reference trajectory");
  }
  controller_status << "Stats" << error_position.norm() << error_velocity.norm()
                    << error_yaw << loop_timer_.average_loop_period();
  VLOG_EVERY_N(1, 20) << "Stats" << error_position.norm()
                      << error_velocity.norm() << error_yaw
                      << loop_timer_.average_loop_period();
  return controller_status;
  return controller_status;
}

void DDPQuadMPCController::setConfig(QuadMPCControllerConfig config) {
  boost::mutex::scoped_lock lock(copy_mutex_);
  config_ = config;
  ddp_config_ = config.ddp_config();
}

void DDPQuadMPCController::outputControl(ControlType &control) {
  // Get Control to return
  control.resize(control_size_);
  control[0] = (9.81 * us_[look_ahead_index_shift_][0]) / kt_[0];
  control.segment<2>(1) =
      xs_[look_ahead_index_shift_].segment<2>(12); // rp_desired
  control[3] = us_[look_ahead_index_shift_][3];    // yaw_rate
}

void DDPQuadMPCController::logData(MPCInputs<StateType> &sensor_data,
                                   ControlType &control) {
  Eigen::Vector3d error_position =
      sensor_data.initial_state.segment<3>(0) - xds_.at(0).segment<3>(0);
  Eigen::Vector3d error_velocity =
      sensor_data.initial_state.segment<3>(6) - xds_.at(0).segment<3>(6);
  DATA_LOG("ddp_quad_mpc_controller")
      << error_position << error_velocity << control << (ddp_->J)
      << Eigen::VectorXd(xds_.at(0).segment<9>(0))
      << loop_timer_.average_loop_period() << DataStream::endl;
}
