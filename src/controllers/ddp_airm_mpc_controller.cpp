#include "aerial_autonomy/controllers/ddp_airm_mpc_controller.h"
#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/log/log.h"
#include <gcop/aerial_manipulation_feedforward_system.h>
#include <gcop/airm_residual_network_model.h>
#include <gcop/load_eigen_matrix.h>

constexpr int DDPAirmMPCController::state_size_,
    DDPAirmMPCController::control_size_;

void DDPAirmMPCController::loadQuadParameters(Eigen::Vector3d &kp_rpy,
                                              Eigen::Vector3d &kd_rpy,
                                              Eigen::VectorXd &p,
                                              std::string folder_path) {
  Eigen::Vector2d kp_rp =
      gcop::loadEigenMatrix(folder_path + "/rpy_gains_kp_0");
  kp_rpy[0] = kp_rp[0];
  kp_rpy[1] = kp_rp[1];
  kp_rpy[2] = 0;
  kd_rpy = gcop::loadEigenMatrix(folder_path + "/rpy_gains_kd_0");
  p << config_.default_thrust_gain();
}

void DDPAirmMPCController::loadArmParameters(Eigen::Vector2d &kp_ja,
                                             Eigen::Vector2d &kd_ja,
                                             std::string folder_path) {
  kp_ja = gcop::loadEigenMatrix(folder_path + "/joint_gains_kp_0");
  kd_ja = gcop::loadEigenMatrix(folder_path + "/joint_gains_kd_0");
}

DDPAirmMPCController::DDPAirmMPCController(
    AirmMPCControllerConfig config,
    std::chrono::duration<double> controller_duration)
    : DDPCasadiMPCController(config.ddp_config(), controller_duration),
      config_(config) {
  // Instantiate system
  std::string folder_path =
      std::string(PROJECT_SOURCE_DIR) + "/" + config.weights_folder();
  Eigen::Vector3d kp_rpy, kd_rpy;
  Eigen::Vector2d kp_ja, kd_ja;
  loadQuadParameters(kp_rpy, kd_rpy, kt_, folder_path);
  loadArmParameters(kp_ja, kd_ja, folder_path);
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
  if (config.use_residual_dynamics()) {
    sys_.reset(new gcop::AirmResidualNetworkModel(
        kt_, kp_rpy, kd_rpy, kp_ja, kd_ja, config.max_joint_velocity(),
        config.n_layers(), folder_path, lb_, ub_, gcop::Activation::tanh,
        config.use_code_generation(), config.mocap_yaw_offset()));
  } else {
    kp_rpy(2) = 0.1; // Toa avoid singularity in GCOP
    sys_.reset(new gcop::AerialManipulationFeedforwardSystem(
        kt_, kp_rpy, kd_rpy, kp_ja, kd_ja, config.max_joint_velocity(), lb_,
        ub_, config.use_code_generation()));
  }
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
  VLOG(1) << "Done setting up ddp";
  // Setting up logger
  DATA_HEADER("ddp_airm_mpc_controller") << "Errorx"
                                         << "Errory"
                                         << "Errorz"
                                         << "Errorja1"
                                         << "Errorja2"
                                         << "thrust_d"
                                         << "rd"
                                         << "pd"
                                         << "yaw_rate_d"
                                         << "Jad1"
                                         << "Jad2"
                                         << "J"
                                         << "Loop timer" << DataStream::endl;
}

DDPAirmMPCController::ControlType DDPAirmMPCController::stationaryControl() {
  Eigen::VectorXd ui(control_size_);
  ui << 1.0, 0, 0, 0, 0, 0;
  return ui;
}

ControllerStatus DDPAirmMPCController::isConvergedImplementation(
    MPCInputs<StateType> sensor_data, GoalType goal) {
  if (!controller_config_status_) {
    LOG(WARNING) << "Controller config invalid!";
    return ControllerStatus(ControllerStatus::Critical);
  }
  boost::mutex::scoped_lock lock(copy_mutex_);
  ControllerStatus controller_status = ControllerStatus::Active;
  if (xds_.at(0).rows() < state_size_) {
    double t = sensor_data.time_since_goal;
    std::pair<StateType, ControlType> state_control_pair = goal->atTime(t);
    xds_.at(0) = state_control_pair.first;
  }
  Eigen::Vector3d error_position =
      sensor_data.initial_state.segment<3>(0) - xds_.at(0).segment<3>(0);
  Eigen::Vector3d error_velocity =
      sensor_data.initial_state.segment<3>(6) - xds_.at(0).segment<3>(6);
  Eigen::Vector2d error_ja =
      sensor_data.initial_state.segment<2>(15) - xds_.at(0).segment<2>(15);
  Eigen::Vector2d error_jv =
      sensor_data.initial_state.segment<2>(17) - xds_.at(0).segment<2>(17);
  if (error_position.squaredNorm() < config_.goal_position_tolerance() *
                                         config_.goal_position_tolerance() &&
      error_velocity.squaredNorm() < config_.goal_velocity_tolerance() *
                                         config_.goal_velocity_tolerance() &&
      error_ja.squaredNorm() < config_.goal_joint_angle_tolerance() *
                                   config_.goal_joint_angle_tolerance() &&
      error_jv.squaredNorm() < config_.goal_joint_velocity_tolerance() *
                                   config_.goal_joint_velocity_tolerance()) {
    VLOG(1) << "Controller Converged!";
    controller_status.setStatus(ControllerStatus::Completed,
                                "Converged to reference trajectory");
  }
  controller_status << "Stats" << error_position.norm() << error_velocity.norm()
                    << error_ja.norm() << error_jv.norm()
                    << loop_timer_.average_loop_period();
  return controller_status;
}

void DDPAirmMPCController::setConfig(AirmMPCControllerConfig config) {
  boost::mutex::scoped_lock lock(copy_mutex_);
  config_ = config;
  ddp_config_ = config.ddp_config();
}

void DDPAirmMPCController::outputControl(ControlType &control) {
  // Get Control to return
  control.resize(control_size_);
  control[0] = (9.81 * us_[look_ahead_index_shift_][0]) / kt_[0];
  control.segment<2>(1) =
      xs_[look_ahead_index_shift_].segment<2>(12); // rp_desired
  control[3] = us_[look_ahead_index_shift_][3];    // yaw_rate
  control.segment<2>(4) =
      xs_[look_ahead_index_shift_].segment<2>(19); // ja_desired
}

void DDPAirmMPCController::logData(MPCInputs<StateType> &sensor_data,
                                   ControlType &control) {
  Eigen::Vector3d error_position =
      sensor_data.initial_state.segment<3>(0) - xds_.at(0).segment<3>(0);
  Eigen::Vector2d error_ja =
      sensor_data.initial_state.segment<2>(15) - xds_.at(0).segment<2>(15);
  DATA_LOG("ddp_airm_mpc_controller")
      << error_position << error_ja << control << (ddp_->J)
      << loop_timer_.average_loop_period() << DataStream::endl;
}
