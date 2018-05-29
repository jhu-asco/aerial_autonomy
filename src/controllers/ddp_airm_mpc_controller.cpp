#include "aerial_autonomy/controllers/ddp_airm_mpc_controller.h"
#include <gcop/load_eigen_matrix.h>

DDPAirmMPCController::DDPAirmMPCController(AirmMPCControllerConfig config)
    : config_(config) {
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
  Eigen::VectorXd kt(1);
  kt << config.default_thrust_gain();
  if (config.use_residual_dynamics()) {
    sys_.reset(new gcop::AirmResidualNetworkModel(
        kt, kp_rpy, kd_rpy, kp_ja, kd_ja, config.max_joint_velocity(),
        config.n_layers(), folder_path, gcop::Activation::tanh, true));
  } else {
    sys_.reset(new gcop::AerialManipulationFeedforwardSystem(
        kt, kp_rpy, kd_rpy, kp_ja, kd_ja, config.max_joint_velocity(), true));
  }
  sys_->instantiateStepFunction();
  // cost
  auto ddp_config = config_.ddp_config();
  int N = ddp_config.n();
  double h = ddp_config.h();
  double tf = h * N;
  std::cout << "Manifold size: " << (sys_->X.n) << std::endl;
  cost_.reset(new gcop::LqCost<Eigen::VectorXd>(*sys_, tf, xf_));
  cost_->Q = (conversions::vectorProtoToEigen(ddp_config.q())).asDiagonal();
  CHECK(cost_->Q.rows() == sys_->X.n)
      << "Cost dimension should be same as system state size";
  cost_->Qf = (conversions::vectorProtoToEigen(ddp_config.qf())).asDiagonal();
  CHECK(cost_->Qf.rows() == sys_->X.n)
      << "Cost dimension should be same as system state size";
  cost_->R = (conversions::vectorProtoToEigen(ddp_config.r())).asDiagonal();
  CHECK(cost_->R.rows() == sys_->U.n)
      << "Control ost dimension should be same as system control size";
  cost_->UpdateGains();
  // Times
  for (int k = 0; k <= N; ++k)
    ts_.push_back(k * h);
  // states
  xs_.resize(N + 1);
  // initial controls
  Eigen::VectorXd ui(6);
  ui << 9.81 / config.default_thrust_gain(), 0, 0, 0, 0, 0;
  us_.resize(N, ui);
  // Ddp
  ddp_.reset(new gcop::Ddp<Eigen::VectorXd>(*sys_, *cost_, ts_, xs_, us_));
  ddp_->mu = config.ddp_config().mu();
  ddp_->debug = config.ddp_config().debug();
}
bool DDPAirmMPCController::runImplementation(
    MPCInputs<StateType> sensor_data,
    ReferenceTrajectory<StateType, ControlType> goal, ControlType &control) {
  return true;
}
