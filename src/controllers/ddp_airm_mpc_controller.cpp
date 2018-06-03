#include "aerial_autonomy/controllers/ddp_airm_mpc_controller.h"
#include "aerial_autonomy/common/atomic.h"
#include <gcop/load_eigen_matrix.h>

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

DDPAirmMPCController::DDPAirmMPCController(AirmMPCControllerConfig config,
                                           double controller_duration)
    : config_(config), kt_(1) {
  // Instantiate system
  std::string folder_path = config.weights_folder();
  Eigen::Vector3d kp_rpy, kd_rpy;
  Eigen::Vector2d kp_ja, kd_ja;
  loadQuadParameters(kp_rpy, kd_rpy, kt_, folder_path);
  loadArmParameters(kp_ja, kd_ja, folder_path);
  if (config.use_residual_dynamics()) {
    sys_.reset(new gcop::AirmResidualNetworkModel(
        kt_, kp_rpy, kd_rpy, kp_ja, kd_ja, config.max_joint_velocity(),
        config.n_layers(), folder_path, gcop::Activation::tanh, true));
  } else {
    sys_.reset(new gcop::AerialManipulationFeedforwardSystem(
        kt_, kp_rpy, kd_rpy, kp_ja, kd_ja, config.max_joint_velocity(), true));
  }
  VLOG(1) << "Instantiating Step function";
  sys_->instantiateStepFunction();
  // cost
  auto ddp_config = config_.ddp_config();
  int N = ddp_config.n();
  double h = ddp_config.h();
  double tf = h * N;
  CHECK(h > 0) << "The time step should be greater than 0";
  control_timer_shift_ =
      std::min(int(std::ceil(controller_duration / h)), N - 1);
  VLOG(1) << "Control timer shift: " << control_timer_shift_;
  look_ahead_index_shift_ = std::ceil(ddp_config.look_ahead_time() / h);
  VLOG(1) << "Look ahead index shift: " << look_ahead_index_shift_;
  CHECK(look_ahead_index_shift_ < N)
      << "Look ahead time should be less than trajectory end time";
  VLOG(1) << "Manifold size: " << (sys_->X.n);
  xf_.resize(21);
  xf_.setZero();
  cost_.reset(new gcop::LqCost<Eigen::VectorXd>(*sys_, tf, xf_));
  VLOG(1) << "Created cost function";
  CHECK(ddp_config.q_size() == sys_->X.n)
      << "Cost dimension should be same as system state size";
  CHECK(ddp_config.qf_size() == sys_->X.n)
      << "Final Cost dimension should be same as system state size";
  CHECK(ddp_config.r_size() == sys_->U.n)
      << "Control cost dimension should be same as system control size";
  cost_->Q =
      (conversions::vectorProtoToEigen(*ddp_config.mutable_q())).asDiagonal();
  cost_->Qf =
      (conversions::vectorProtoToEigen(*ddp_config.mutable_qf())).asDiagonal();
  cost_->R =
      (conversions::vectorProtoToEigen(*ddp_config.mutable_r())).asDiagonal();
  cost_->UpdateGains();
  // References:
  VLOG(1) << "Trajectory length: " << N;
  xds_.resize(N + 1);
  uds_.resize(N);
  cost_->SetReference(&xds_, &uds_);
  // Times
  for (int k = 0; k <= N; ++k) {
    ts_.push_back(k * h);
  }
  // states
  Eigen::VectorXd default_state(21);
  default_state.setZero();
  xs_.resize(N + 1, default_state);
  resetControls(); // Set controls to default values
  // Ddp
  VLOG(1) << "Creating ddp";
  ddp_.reset(
      new gcop::Ddp<Eigen::VectorXd>(*sys_, *cost_, ts_, xs_, us_, &kt_));
  ddp_->mu = config.ddp_config().mu();
  ddp_->debug = config.ddp_config().debug();
  VLOG(1) << "Done setting up ddp";
}

void DDPAirmMPCController::resetControls() {
  VLOG(1) << "Resetting Controls";
  // initial controls
  Eigen::VectorXd ui(6);
  ui << 9.81 / config_.default_thrust_gain(), 0, 0, 0, 0, 0;
  us_.resize(config_.ddp_config().n(), ui);
  if (ddp_) {
    ddp_->Update();
  }
}

ControllerStatus DDPAirmMPCController::isConvergedImplementation(
    MPCInputs<StateType> sensor_data, GoalType) {
  boost::mutex::scoped_lock lock(copy_mutex_);
  ControllerStatus controller_status = ControllerStatus::Active;
  Eigen::Vector3d error_position =
      sensor_data.initial_state.segment<3>(0) - xds_.at(0).segment<3>(0);
  Eigen::Vector3d error_velocity =
      sensor_data.initial_state.segment<3>(6) - xds_.at(0).segment<3>(6);
  if (error_position.squaredNorm() < config_.goal_position_tolerance() *
                                         config_.goal_position_tolerance() &&
      error_velocity.squaredNorm() < config_.goal_velocity_tolerance() *
                                         config_.goal_velocity_tolerance()) {
    controller_status.setStatus(ControllerStatus::Completed,
                                "Converged to reference trajectory");
  }
  controller_status << "Error Position" << error_position(0)
                    << error_position(1) << error_position(2)
                    << error_velocity(0) << error_velocity(1)
                    << error_velocity(2);
  return controller_status;
}

void DDPAirmMPCController::rotateControls(int shift_length) {
  int N = us_.size();
  // Rotate controls by control timer shift
  // in proto file (Default 50 Hz)
  for (int i = 0; i < N - shift_length; ++i) {
    us_[i] = us_[i + shift_length];
  }
  for (int i = N - shift_length; i < N; ++i) {
    us_[i] = us_[N - 1];
  }
}

bool DDPAirmMPCController::runImplementation(MPCInputs<StateType> sensor_data,
                                             GoalType goal,
                                             ControlType &control) {
  bool result = true;
  loop_timer_.loop_start();
  boost::mutex::scoped_lock lock(copy_mutex_);
  auto &ddp_config = config_.ddp_config();
  int max_iters = ddp_config.max_iters();
  int N = ddp_config.n();
  double h = ddp_config.h();
  double t0 = sensor_data.time_since_goal;
  // Get MPC Reference from high level reference trajectory
  for (int i = 0; i <= N; ++i) {
    double t = t0 + i * h;
    std::pair<StateType, ControlType> state_control_pair = goal->atTime(t);
    xds_.at(i) = state_control_pair.first;
    if (i < N) {
      uds_.at(i) = state_control_pair.second;
    }
  }
  // Start state
  xs_.at(0) = sensor_data.initial_state;
  // Parameters
  kt_[0] = sensor_data.parameters[0]; // copy kt
  rotateControls(control_timer_shift_);
  // Update states based on controls
  ddp_->Update();
  double J = 1e6; // Assume start cost is some large value
  // Run MPC Iterations
  for (int i = 0; i < max_iters; ++i) {
    ddp_->Iterate();
    VLOG(3) << "DDP_J: " << (ddp_->J);
    // Check for convergence
    if (std::abs(ddp_->J - J) < ddp_config.min_cost_decrease()) {
      VLOG(3) << "Converged";
      break;
    }
    J = ddp_->J;
  }
  VLOG(3) << "DDP_J: " << (ddp_->J);
  VLOG(3) << "xf: " << xs_.back().transpose();
  VLOG(3) << "xd: " << xds_.back().transpose();
  if (ddp_->J > ddp_config.min_cost()) {
    LOG(WARNING) << "Failed to get a reasonable trajectory using Ddp. J: "
                 << (ddp_->J);
    result = false;
  }
  // Get Control to return
  control = us_[look_ahead_index_shift_];
  loop_timer_.loop_end();
  return result;
}

void DDPAirmMPCController::getTrajectory(std::vector<StateType> &xs,
                                         std::vector<ControlType> &us) const {
  boost::mutex::scoped_lock lock(copy_mutex_);
  xs = xs_;
  us = us_;
}

void DDPAirmMPCController::getDesiredTrajectory(
    std::vector<StateType> &xds, std::vector<ControlType> &uds) const {
  boost::mutex::scoped_lock lock(copy_mutex_);
  xds = xds_;
  uds = uds_;
}
