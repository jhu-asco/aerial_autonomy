#include "aerial_autonomy/controllers/ddp_airm_mpc_controller.h"
#include "aerial_autonomy/common/atomic.h"
#include <gcop/load_eigen_matrix.h>

DDPAirmMPCController::DDPAirmMPCController(AirmMPCControllerConfig config,
                                           double controller_duration)
    : config_(config), kt_(1) {
  // Instantiate system
  std::string folder_path = config.weights_folder();
  Eigen::Vector2d kp_rp =
      gcop::loadEigenMatrix(folder_path + "/rpy_gains_kp_0");
  Eigen::Vector3d kp_rpy;
  kp_rpy[0] = kp_rp[0];
  kp_rpy[1] = kp_rp[1];
  kp_rpy[2] = 0;
  Eigen::Vector3d kd_rpy =
      gcop::loadEigenMatrix(folder_path + "/rpy_gains_kd_0");
  Eigen::Vector2d kp_ja =
      gcop::loadEigenMatrix(folder_path + "/joint_gains_kp_0");
  Eigen::Vector2d kd_ja =
      gcop::loadEigenMatrix(folder_path + "/joint_gains_kd_0");
  kt_ << config.default_thrust_gain();
  if (config.use_residual_dynamics()) {
    sys_.reset(new gcop::AirmResidualNetworkModel(
        kt_, kp_rpy, kd_rpy, kp_ja, kd_ja, config.max_joint_velocity(),
        config.n_layers(), folder_path, gcop::Activation::tanh, true));
  } else {
    sys_.reset(new gcop::AerialManipulationFeedforwardSystem(
        kt_, kp_rpy, kd_rpy, kp_ja, kd_ja, config.max_joint_velocity(), true));
  }
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
  cost_.reset(new gcop::LqCost<Eigen::VectorXd>(*sys_, tf, xf_));
  cost_->Q = (conversions::vectorProtoToEigen(ddp_config.q())).asDiagonal();
  CHECK(cost_->Q.rows() == sys_->X.n)
      << "Cost dimension should be same as system state size";
  cost_->Qf = (conversions::vectorProtoToEigen(ddp_config.qf())).asDiagonal();
  CHECK(cost_->Qf.rows() == sys_->X.n)
      << "Cost dimension should be same as system state size";
  cost_->R = (conversions::vectorProtoToEigen(ddp_config.r())).asDiagonal();
  CHECK(cost_->R.rows() == sys_->U.n)
      << "Control cost dimension should be same as system control size";
  cost_->UpdateGains();
  // References:
  xds_.resize(N + 1);
  uds_.resize(N);
  cost_->SetReference(&xds_, &uds_);
  // Times
  for (int k = 0; k <= N; ++k)
    ts_.push_back(k * h);
  // states
  xs_.resize(N + 1);
  resetControls(); // Set controls to default values
  // Ddp
  ddp_.reset(
      new gcop::Ddp<Eigen::VectorXd>(*sys_, *cost_, ts_, xs_, us_, &kt_));
  ddp_->mu = config.ddp_config().mu();
  ddp_->debug = config.ddp_config().debug();
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

bool DDPAirmMPCController::runImplementation(MPCInputs<StateType> sensor_data,
                                             GoalType goal,
                                             ControlType &control) {
  loop_timer_.loop_start();
  boost::mutex::scoped_lock lock(copy_mutex_);
  auto &ddp_config = config_.ddp_config();
  int max_iters = ddp_config.max_iters();
  int N = ddp_config.n();
  double h = ddp_config.h();
  double t0 = sensor_data.time_since_goal;
  // Get MPC Reference from high level reference trajectory
  for (int i = 0; i < N; ++i) {
    double t = t0 + i * h;
    std::pair<StateType, ControlType> state_control_pair = goal->atTime(t);
    xds_.at(i) = state_control_pair.first;
    uds_.at(i) = state_control_pair.second;
  }
  // Start state
  xs_.at(0) = sensor_data.initial_state;
  // Parameters
  kt_[0] = sensor_data.parameters[0]; // copy kt
  // Rotate controls by control timer shift
  // in proto file (Default 50 Hz)
  for (int i = 0; i < N - control_timer_shift_; ++i) {
    us_[i] = us_[i + control_timer_shift_];
  }
  for (int i = N - control_timer_shift_; i < N; ++i) {
    us_[i] = us_[N - 1];
  }
  // Update states based on controls
  ddp_->Update();
  double J = 1e6; // Assume start cost is some large value
  // Run MPC Iterations
  for (int i = 0; i < max_iters; ++i) {
    ddp_->Iterate();
    // Check for convergence
    if (std::abs(ddp_->J - J) < ddp_config.min_cost_decrease()) {
      break;
    }
  }
  VLOG(3) << "DDP_J: " << (ddp_->J);
  VLOG(3) << "xf: " << xs_.back().transpose();
  VLOG(3) << "xd: " << xds_.back().transpose();
  if (ddp_->J > ddp_config.min_cost()) {
    LOG(WARNING) << "Failed to get a reasonable trajectory using Ddp. J: "
                 << (ddp_->J);
    return false;
  }
  // Get Control to return
  control = us_[look_ahead_index_shift_];
  loop_timer_.loop_end();
  return true;
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
