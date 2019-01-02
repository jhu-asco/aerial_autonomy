#include <aerial_autonomy/common/conversions.h>
#include <aerial_autonomy/common/math.h>
#include <aerial_autonomy/controller_connectors/mpc_controller_airm_connector.h>
#include <aerial_autonomy/log/log.h>

constexpr int MPCControllerAirmConnector::state_size_;

MPCControllerAirmConnector::MPCControllerAirmConnector(
    parsernode::Parser &drone_hardware, ArmParser &arm_hardware,
    AbstractMPCController<StateType, ControlType> &controller,
    ThrustGainEstimator &thrust_gain_estimator, int delay_buffer_size,
    MPCConnectorConfig config,
    SensorPtr<std::pair<tf::StampedTransform, tf::Vector3>> odom_sensor,
    AbstractConstraintGeneratorPtr constraint_generator)
    : BaseMPCControllerQuadConnector(drone_hardware, controller,
                                     thrust_gain_estimator, delay_buffer_size,
                                     config, odom_sensor, constraint_generator),
      arm_hardware_(arm_hardware), joint_angle_commands_(2),
      joint_velocity_filter_(config.joint_velocity_exp_gain()),
      previous_joint_measurements_initialized_(false) {
  clearJointCommandBuffers();
  // clang-format off
  DATA_HEADER("airm_mpc_state_estimator") << "x" << "y" << "z"
                                          << "r" << "p" << "y"
                                          << "vx" << "vy" << "vz"
                                          << "rdot" << "pdot" << "ydot"
                                          << "rd" << "pd" << "yd"
                                          << "ja1" << "ja2"
                                          << "jv1" << "jv2"
                                          << "jad1" << "jad2"
                                          << "kt" << "bias_r" << "bias_p"
                                          << DataStream::endl;
  // clang-format on
}

void MPCControllerAirmConnector::initialize() {
  previous_measurement_time_ = std::chrono::high_resolution_clock::now();
  previous_joint_measurements_initialized_ = false;
  joint_velocity_filter_.reset();
  clearJointCommandBuffers();
  initializePrivateController(private_controller_);
}

void MPCControllerAirmConnector::clearJointCommandBuffers() {
  auto joint_angles = arm_hardware_.getJointAngles();
  if (joint_angles.size() == 2) {
    previous_joint_commands_ =
        Eigen::Vector2d(joint_angles.at(0), joint_angles.at(1));
  } else {
    // This check is only needed to fix tests where arm hardware
    // is giving out empty joint angles
    LOG(WARNING) << "The joint angles from arm hardware are not 2";
    previous_joint_commands_ = Eigen::Vector2d(0, 0);
  }
}

void MPCControllerAirmConnector::sendControllerCommands(ControlType control) {
  BaseMPCControllerQuadConnector::sendControllerCommands(control);
  joint_angle_commands_.at(0) = control(4);
  joint_angle_commands_.at(1) = control(5);
  arm_hardware_.setJointAngles(joint_angle_commands_);
  previous_joint_commands_ = control.segment<2>(4);
}

double MPCControllerAirmConnector::getTimeDiff() {
  // Timing logic
  auto current_time = std::chrono::high_resolution_clock::now();
  double dt =
      std::chrono::duration<double>(current_time - previous_measurement_time_)
          .count();
  if (config_.use_perfect_time_diff()) {
    dt = config_.perfect_time_diff();
  }
  previous_measurement_time_ = current_time;
  if (dt < 1e-4) {
    LOG(WARNING) << "Time diff cannot be smaller than 1e-4";
    dt = 1e-4;
  }
  return dt;
}

bool MPCControllerAirmConnector::estimateStateAndParameters(
    Eigen::VectorXd &current_state, Eigen::VectorXd &params) {
  double dt = getTimeDiff();
  current_state.resize(state_size_);
  // Joint angles
  std::vector<double> joint_angles = arm_hardware_.getJointAngles();
  Eigen::Vector2d joint_angles_vec(joint_angles.at(0), joint_angles.at(1));
  // Differentiate:
  Eigen::Vector2d joint_velocities;
  if (previous_joint_measurements_initialized_) {
    joint_velocities = (joint_angles_vec - previous_joint_angles_) / dt;
  } else {
    joint_velocities = Eigen::Vector2d::Zero();
    previous_joint_measurements_initialized_ = true;
  }
  // Update filtered velocities:
  Eigen::Vector2d filtered_joint_velocity =
      joint_velocity_filter_.addAndFilter(joint_velocities);
  // Fill state
  current_state.segment<2>(15) = joint_angles_vec;
  current_state.segment<2>(17) = filtered_joint_velocity;
  current_state.segment<2>(19) = previous_joint_commands_;
  // Fill previous measurements
  previous_joint_angles_ = joint_angles_vec;
  // Fill Quad stuff
  bool result = fillQuadStateAndParameters(current_state, params);
  if (result) {
    DATA_LOG("airm_mpc_state_estimator")
        << current_state << params[0]
        << thrust_gain_estimator_.getRollPitchBias() << DataStream::endl;
  }
  return result;
}
