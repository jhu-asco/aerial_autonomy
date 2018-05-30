#include <aerial_autonomy/common/conversions.h>
#include <aerial_autonomy/common/math.h>
#include <aerial_autonomy/controller_connectors/mpc_controller_airm_connector.h>

MPCControllerAirmConnector::MPCControllerAirmConnector(
    parsernode::Parser &drone_hardware, ArmParser &arm_hardware,
    AbstractMPCController<StateType, ControlType> &controller,
    Sensor<tf::Transform> &pose_sensor,
    ThrustGainEstimator &thrust_gain_estimator,
    AbstractConstraintGenerator &constraint_generator, int delay_buffer_size)
    : MPCControllerConnector(controller, constraint_generator,
                             ControllerGroup::UAV),
      drone_hardware_(drone_hardware), arm_hardware_(arm_hardware),
      pose_sensor_(pose_sensor), thrust_gain_estimator_(thrust_gain_estimator),
      joint_angle_commands_(2), previous_measurements_(8),
      previous_measurement_time_(std::chrono::high_resolution_clock::now()),
      delay_buffer_size_(delay_buffer_size) {
  clearCommandBuffers();
}

void MPCControllerAirmConnector::clearCommandBuffers() {
  previous_joint_commands_.setZero();
  std::queue<Eigen::Vector3d> zero_queue;
  for (int i = 0; i < delay_buffer_size_; ++i) {
    zero_queue.push(Eigen::Vector3d::Zero());
  }
  rpy_command_buffer_.swap(zero_queue);
}

void MPCControllerAirmConnector::sendControllerCommands(ControlType control) {
  geometry_msgs::Quaternion rpyt_msg;
  rpyt_msg.x = control(1);
  rpyt_msg.y = control(2);
  rpyt_msg.z = control(3);
  rpyt_msg.w = control(0);
  drone_hardware_.cmdrpyawratethrust(rpyt_msg);
  joint_angle_commands_.at(0) = control(4);
  joint_angle_commands_.at(1) = control(5);
  arm_hardware_.setJointAngles(joint_angle_commands_);
  previous_joint_commands_ = control.segment<2>(4);
  rpy_command_buffer_.pop();
  rpy_command_buffer_.push(control.segment<3>(1));
}

void MPCControllerAirmConnector::setGoal(
    ReferenceTrajectoryPtr<StateType, ControlType> goal) {
  MPCControllerConnector<Eigen::VectorXd, Eigen::VectorXd>::setGoal(goal);
  VLOG(1) << "Clearing thrust estimator buffer";
  thrust_gain_estimator_.clearBuffer();
  clearCommandBuffers();
}

bool MPCControllerAirmConnector::estimateStateAndParameters(
    Eigen::VectorXd &current_state, Eigen::VectorXd &params) {
  current_state.resize(21);
  auto current_time = std::chrono::high_resolution_clock::now();
  double dt =
      std::chrono::duration<double>(current_time - previous_measurement_time_)
          .count();
  if (dt < 1e-4) {
    LOG(WARNING) << "Time diff cannot be smaller than 1e-4";
    return false;
  }
  ///\todo Do some filtering on position, rpy before differentiation
  tf::Transform quad_pose = pose_sensor_.getSensorData();
  // Position
  const auto &quad_position = quad_pose.getOrigin();
  Eigen::Vector3d p(quad_position.x(), quad_position.y(), quad_position.z());
  // Euler angles
  Eigen::Vector3d rpy = conversions::transformTfToRPY(quad_pose);
  // Joint angles
  std::vector<double> joint_angles = arm_hardware_.getJointAngles();
  Eigen::Vector2d joint_angles_vec(joint_angles.at(0), joint_angles.at(1));
  // Differentiate:
  Eigen::Vector3d v, delta_rpy;
  Eigen::Vector2d joint_velocities;
  if (previous_measurements_initialized_) {
    v = (p - previous_measurements_.segment<3>(0)) / dt;
    delta_rpy = rpy - previous_measurements_.segment<3>(3);
    delta_rpy[2] = math::angleWrap(delta_rpy[2]);
    joint_velocities =
        (joint_angles_vec - previous_measurements_.segment<2>(6)) / dt;
  } else {
    v = delta_rpy = Eigen::Vector3d::Zero();
    joint_velocities = Eigen::Vector2d::Zero();
    previous_measurements_initialized_ = true;
  }
  // Fill previous measurements and time
  previous_measurements_.segment<6>(0) = current_state.segment<6>(0);
  previous_measurements_.segment<2>(6) = joint_angles_vec;
  previous_measurement_time_ = current_time;

  // Fill state
  current_state.segment<3>(0) = p;
  current_state.segment<3>(3) = rpy;
  current_state.segment<3>(6) = v;
  current_state.segment<3>(9) = delta_rpy / dt;
  current_state.segment<3>(12) = rpy_command_buffer_.front();
  current_state.segment<2>(15) = joint_angles_vec;
  current_state.segment<2>(17) = joint_velocities;
  current_state.segment<2>(19) = previous_joint_commands_;
  // Estimate thrust gain parameter
  parsernode::common::quaddata quad_data;
  drone_hardware_.getquaddata(quad_data);
  thrust_gain_estimator_.addSensorData(quad_data.rpydata.x, quad_data.rpydata.y,
                                       quad_data.linacc.z);
  params.resize(1, thrust_gain_estimator_.getThrustGain());
  return true;
}
