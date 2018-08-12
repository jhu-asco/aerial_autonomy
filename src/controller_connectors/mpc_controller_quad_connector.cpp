#include <aerial_autonomy/common/conversions.h>
#include <aerial_autonomy/common/math.h>
#include <aerial_autonomy/controller_connectors/mpc_controller_quad_connector.h>
#include <aerial_autonomy/log/log.h>

constexpr int MPCControllerQuadConnector::state_size_;

MPCControllerQuadConnector::MPCControllerQuadConnector(
    parsernode::Parser &drone_hardware,
    AbstractMPCController<StateType, ControlType> &controller,
    ThrustGainEstimator &thrust_gain_estimator, int delay_buffer_size,
    MPCConnectorConfig config, SensorPtr<tf::StampedTransform> pose_sensor,
    AbstractConstraintGeneratorPtr constraint_generator)
    : QuadAirmMPCCommonConnector(drone_hardware, controller,
                                 thrust_gain_estimator, delay_buffer_size,
                                 config, pose_sensor, constraint_generator),
      previous_measurements_(3) {
  // clang-format off
  DATA_HEADER("quad_mpc_state_estimator") << "x" << "y" << "z"
                                          << "vx" << "vy" << "vz"
                                          << "r" << "p" << "y"
                                          << "rdot" << "pdot" << "ydot"
                                          << "rd" << "pd" << "yd"
                                          << "kt" << DataStream::endl;
  // clang-format on
}

void MPCControllerQuadConnector::initialize() {
  MPCControllerConnector::initialize();
  VLOG(1) << "Clearing thrust estimator buffer";
  thrust_gain_estimator_.clearBuffer();
  previous_measurements_initialized_ = false;
  private_controller_.resetControls();
  clearCommandBuffers();
  int iters = private_controller_.getMaxIters();
  private_controller_.setMaxIters(100);
  run();
  private_controller_.setMaxIters(iters);
}

bool MPCControllerQuadConnector::estimateStateAndParameters(
    Eigen::VectorXd &current_state, Eigen::VectorXd &params) {
  current_state.resize(state_size_);
  // Timing logic
  auto current_time = std::chrono::high_resolution_clock::now();
  double dt =
      std::chrono::duration<double>(current_time - previous_measurement_time_)
          .count();
  if (config_.use_perfect_time_diff()) {
    dt = config_.perfect_time_diff();
  }
  if (dt < 1e-4) {
    LOG(WARNING) << "Time diff cannot be smaller than 1e-4";
    return false;
  }
  // Get Quad data
  parsernode::common::quaddata quad_data;
  drone_hardware_.getquaddata(quad_data);
  ///\todo Do some filtering on position, rpy before differentiation
  tf::Transform quad_pose;
  if (pose_sensor_) {
    if (pose_sensor_->getSensorStatus() != SensorStatus::VALID) {
      LOG(WARNING) << "Sensor invalid";
      return false;
    }
    quad_pose = pose_sensor_->getSensorData();
  } else {
    quad_pose = conversions::getPose(quad_data);
  }
  // Position
  const auto &quad_position = quad_pose.getOrigin();
  Eigen::Vector3d p(quad_position.x(), quad_position.y(), quad_position.z());
  Eigen::Vector3d rpy;
  Eigen::Vector3d omega(quad_data.omega.x, quad_data.omega.y,
                        quad_data.omega.z);
  // Euler angles
  if (pose_sensor_) {
    rpy = conversions::transformTfToRPY(quad_pose);
  } else {
    rpy = Eigen::Vector3d(quad_data.rpydata.x, quad_data.rpydata.y,
                          quad_data.rpydata.z);
  }
  // Differentiate:
  Eigen::Vector3d v;
  if (previous_measurements_initialized_) {
    v = (p - previous_measurements_) / dt;
  } else {
    v = Eigen::Vector3d::Zero();
    filtered_rpydot_ = Eigen::Vector3d::Zero();
    filtered_velocity_ = Eigen::Vector3d::Zero();
    previous_measurements_initialized_ = true;
  }
  double rpydot_gain = config_.rpydot_gain();
  // Get rpydot from omega:
  filtered_rpydot_ =
      (rpydot_gain * filtered_rpydot_ +
       (1 - rpydot_gain) * conversions::omegaToRpyDot(omega, rpy));
  // Update filtered velocities:
  double velocity_exp_gain = config_.velocity_exp_gain();
  filtered_velocity_ =
      velocity_exp_gain * filtered_velocity_ + (1 - velocity_exp_gain) * v;
  // Fill state
  current_state.segment<3>(0) = p;
  current_state.segment<3>(3) = rpy;
  current_state.segment<3>(6) = filtered_velocity_;
  current_state.segment<3>(9) = filtered_rpydot_;
  current_state.segment<3>(12) = rpy_command_buffer_.front();
  // Fill previous measurements and time
  previous_measurements_ = current_state.segment<3>(0);
  previous_measurement_time_ = current_time;
  // Estimate thrust gain parameter
  thrust_gain_estimator_.addSensorData(quad_data.rpydata.x, quad_data.rpydata.y,
                                       quad_data.linacc.z);
  params.resize(1);
  params << thrust_gain_estimator_.getThrustGain();
  DATA_LOG("quad_mpc_state_estimator") << current_state << params[0]
                                       << DataStream::endl;
  return true;
}
