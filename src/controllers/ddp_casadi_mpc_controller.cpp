#include "aerial_autonomy/controllers/ddp_casadi_mpc_controller.h"

DDPCasadiMPCController::DDPCasadiMPCController(
    DDPMPCControllerConfig ddp_config,
    std::chrono::duration<double> controller_duration)
    : ddp_config_(ddp_config), kt_(1), look_ahead_index_shift_(1),
      controller_config_status_(true) {
  // parameters from ddp config
  unsigned int N = ddp_config.n();
  double h = ddp_config.h();
  CHECK(h > 0) << "The time step should be greater than 0";
  control_timer_shift_ =
      std::min(uint(std::ceil(controller_duration.count() / h)), N - 1);
  VLOG(1) << "Control timer shift: " << control_timer_shift_;
  max_look_ahead_index_shift_ =
      uint(std::ceil(ddp_config.look_ahead_time() / h));
  VLOG(1) << "Look ahead index shift: " << max_look_ahead_index_shift_;
  CHECK(max_look_ahead_index_shift_ < N)
      << "Look ahead time should be less than trajectory end time";
  // References:
  VLOG(1) << "Trajectory length: " << N;
  xds_.resize(N + 1);
  uds_.resize(N);
  // Times
  for (unsigned int k = 0; k <= N; ++k) {
    ts_.push_back(k * h);
  }
  max_iters_ = ddp_config.max_iters();
}

void DDPCasadiMPCController::resetDDP() {
  // Ddp
  VLOG(1) << "Creating ddp";
  ddp_.reset(
      new gcop::Ddp<Eigen::VectorXd>(*sys_, *cost_, ts_, xs_, us_, &kt_));
  ddp_->mu = ddp_config_.mu();
  ddp_->debug = ddp_config_.debug();
}

void DDPCasadiMPCController::resetControls() {
  if (!controller_config_status_) {
    LOG(WARNING) << "Controller config invalid!";
    return;
  }
  VLOG(1) << "Resetting Controls";
  // initial controls
  Eigen::VectorXd ui = stationaryControl();
  unsigned int N = ddp_config_.n();
  us_.resize(N, ui);
  resetDDP();
  ddp_->Update();
  look_ahead_index_shift_ = 1;
}

void DDPCasadiMPCController::setMaxIters(int iters) {
  CHECK(iters >= 1) << "Number of iters should be greater than 1";
  max_iters_ = iters;
}

int DDPCasadiMPCController::getMaxIters() const { return max_iters_; }

void DDPCasadiMPCController::rotateControls(unsigned int shift_length) {
  unsigned long N = us_.size();
  // Rotate controls by control timer shift
  // in proto file (Default 50 Hz)
  for (unsigned int i = 0; i < N - shift_length; ++i) {
    us_[i] = us_[i + shift_length];
  }
  for (unsigned long i = N - shift_length; i < N; ++i) {
    us_[i] = us_[N - 1];
  }
}

bool DDPCasadiMPCController::runImplementation(MPCInputs<StateType> sensor_data,
                                               GoalType goal,
                                               ControlType &control) {
  if (!controller_config_status_) {
    LOG(WARNING) << "Controller config invalid!";
    return false;
  }
  bool result = true;
  loop_timer_.loop_start();
  boost::mutex::scoped_lock lock(copy_mutex_);
  unsigned int N = ddp_config_.n();
  double h = ddp_config_.h();
  double t0 = sensor_data.time_since_goal;
  // Get MPC Reference from high level reference trajectory
  for (unsigned int i = 0; i <= N; ++i) {
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
  for (unsigned int i = 0; i < max_iters_; ++i) {
    ddp_->Iterate();
    // Check for convergence
    if (std::abs(ddp_->J - J) < ddp_config_.min_cost_decrease()) {
      VLOG(5) << "Converged";
      break;
    }
    J = ddp_->J;
  }
  if (ddp_->J > ddp_config_.max_cost()) {
    LOG(WARNING) << "Failed to get a reasonable trajectory using Ddp. J: "
                 << (ddp_->J);
    result = false;
  }
  // Get Control to return
  outputControl(control);
  look_ahead_index_shift_ =
      std::min(look_ahead_index_shift_ + 1, max_look_ahead_index_shift_);
  loop_timer_.loop_end();
  VLOG(5) << "T0: " << t0;
  VLOG(5) << "DDP_J: " << (ddp_->J);
  VLOG(5) << "xs0: " << xs_.front().transpose();
  VLOG(5) << "xf: " << xs_.back().transpose();
  VLOG(5) << "xd: " << xds_.back().transpose();
  VLOG(5) << "u: " << control.transpose();

  logData(sensor_data, control);
  return result;
}

void DDPCasadiMPCController::getTrajectory(std::vector<StateType> &xs,
                                           std::vector<ControlType> &us) const {
  boost::mutex::scoped_lock lock(copy_mutex_);
  xs = xs_;
  us = us_;
}

void DDPCasadiMPCController::getDesiredTrajectory(
    std::vector<StateType> &xds, std::vector<ControlType> &uds) const {
  boost::mutex::scoped_lock lock(copy_mutex_);
  xds = xds_;
  uds = uds_;
}
