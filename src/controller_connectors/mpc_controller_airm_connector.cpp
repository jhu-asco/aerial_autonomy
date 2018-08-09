#include <aerial_autonomy/common/conversions.h>
#include <aerial_autonomy/common/math.h>
#include <aerial_autonomy/controller_connectors/mpc_controller_airm_connector.h>
#include <aerial_autonomy/log/log.h>

constexpr int MPCControllerAirmConnector::state_size_;

MPCControllerAirmConnector::MPCControllerAirmConnector(
    parsernode::Parser &drone_hardware, ArmParser &arm_hardware,
    AbstractMPCController<StateType, ControlType> &controller,
    ThrustGainEstimator &thrust_gain_estimator, int delay_buffer_size,
    SensorPtr<tf::StampedTransform> pose_sensor,
    AbstractConstraintGeneratorPtr constraint_generator)
    : QuadAirmMPCCommonConnector(drone_hardware, controller,
                                 thrust_gain_estimator, delay_buffer_size,
                                 pose_sensor, constraint_generator),
      arm_hardware_(arm_hardware), joint_angle_commands_(2),
      previous_measurements_(5) {
  clearJointCommandBuffers();
  // clang-format off
  DATA_HEADER("airm_mpc_state_estimator") << "x" << "y" << "z"
                                          << "vx" << "vy" << "vz"
                                          << "r" << "p" << "y"
                                          << "rdot" << "pdot" << "ydot"
                                          << "rd" << "pd" << "yd"
                                          << "ja1" << "ja2"
                                          << "jv1" << "jv2"
                                          << "jad1" << "jad2"
                                          << "kt" << DataStream::endl;
  // clang-format on
}

void MPCControllerAirmConnector::initialize() {
  MPCControllerConnector::initialize();
  VLOG(1) << "Clearing thrust estimator buffer";
  thrust_gain_estimator_.clearBuffer();
  previous_measurements_initialized_ = false;
  private_controller_.resetControls();
  clearCommandBuffers();
  clearJointCommandBuffers();
  int iters = private_controller_.getMaxIters();
  private_controller_.setMaxIters(100);
  run();
  private_controller_.setMaxIters(iters);
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
  QuadAirmMPCCommonConnector::sendControllerCommands(control);
  joint_angle_commands_.at(0) = control(4);
  joint_angle_commands_.at(1) = control(5);
  arm_hardware_.setJointAngles(joint_angle_commands_);
  previous_joint_commands_ = control.segment<2>(4);
}

bool MPCControllerAirmConnector::estimateStateAndParameters(
    Eigen::VectorXd &current_state, Eigen::VectorXd &params) {
  current_state.resize(state_size_);
  // Timing logic
  auto current_time = std::chrono::high_resolution_clock::now();
  double dt =
      std::chrono::duration<double>(current_time - previous_measurement_time_)
          .count();
  if (use_perfect_time_diff_) {
    dt = perfect_time_diff_;
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
  // Joint angles
  std::vector<double> joint_angles = arm_hardware_.getJointAngles();
  Eigen::Vector2d joint_angles_vec(joint_angles.at(0), joint_angles.at(1));
  // Differentiate:
  Eigen::Vector3d v;
  Eigen::Vector2d joint_velocities;
  if (previous_measurements_initialized_) {
    v = (p - previous_measurements_.segment<3>(0)) / dt;
    joint_velocities =
        (joint_angles_vec - previous_measurements_.segment<2>(3)) / dt;
  } else {
    v = Eigen::Vector3d::Zero();
    joint_velocities = Eigen::Vector2d::Zero();
    filtered_joint_velocity_ = Eigen::Vector2d::Zero();
    filtered_rpydot_ = Eigen::Vector3d::Zero();
    filtered_velocity_ = Eigen::Vector3d::Zero();
    previous_measurements_initialized_ = true;
  }
  double rpydot_gain_ = 0.7;
  // Get rpydot from omega:
  filtered_rpydot_ =
      (rpydot_gain_ * filtered_rpydot_ +
       (1 - rpydot_gain_) * conversions::omegaToRpyDot(omega, rpy));
  // Update filtered velocities:
  filtered_joint_velocity_ = angular_exp_gain_ * filtered_joint_velocity_ +
                             (1 - angular_exp_gain_) * joint_velocities;
  filtered_velocity_ =
      velocity_exp_gain_ * filtered_velocity_ + (1 - velocity_exp_gain_) * v;
  // filtered_joint_velocity_ =
  //    joint_velocities; //// If using filter then uncomment above line
  // Fill state
  current_state.segment<3>(0) = p;
  current_state.segment<3>(3) = rpy;
  current_state.segment<3>(6) = filtered_velocity_;
  current_state.segment<3>(9) = filtered_rpydot_;
  current_state.segment<3>(12) = rpy_command_buffer_.front();
  current_state.segment<2>(15) = joint_angles_vec;
  current_state.segment<2>(17) = filtered_joint_velocity_;
  current_state.segment<2>(19) = previous_joint_commands_;
  // Fill previous measurements and time
  previous_measurements_.segment<3>(0) = current_state.segment<3>(0);
  previous_measurements_.segment<2>(3) = joint_angles_vec;
  previous_measurement_time_ = current_time;
  // Estimate thrust gain parameter
  thrust_gain_estimator_.addSensorData(quad_data.rpydata.x, quad_data.rpydata.y,
                                       quad_data.linacc.z);
  params.resize(1);
  params << thrust_gain_estimator_.getThrustGain();
  DATA_LOG("airm_mpc_state_estimator") << current_state << params[0]
                                       << DataStream::endl;
  return true;
}
