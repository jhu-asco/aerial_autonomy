#include <aerial_autonomy/common/conversions.h>
#include <aerial_autonomy/common/math.h>
#include <aerial_autonomy/controller_connectors/base_mpc_controller_quad_connector.h>
#include <aerial_autonomy/log/log.h>

BaseMPCControllerQuadConnector::BaseMPCControllerQuadConnector(
    parsernode::Parser &drone_hardware,
    AbstractMPCController<StateType, ControlType> &controller,
    ThrustGainEstimator &thrust_gain_estimator, int delay_buffer_size,
    MPCConnectorConfig config, SensorPtr<tf::StampedTransform> pose_sensor,
    AbstractConstraintGeneratorPtr constraint_generator)
    : MPCControllerConnector(controller, ControllerGroup::UAV,
                             constraint_generator),
      drone_hardware_(drone_hardware), pose_sensor_(pose_sensor),
      thrust_gain_estimator_(thrust_gain_estimator),
      previous_measurement_time_(std::chrono::high_resolution_clock::now()),
      previous_measurements_initialized_(false), previous_measurements_(3),
      delay_buffer_size_(delay_buffer_size), private_controller_(controller),
      config_(config) {
  clearCommandBuffers();
}

void BaseMPCControllerQuadConnector::clearCommandBuffers() {
  parsernode::common::quaddata quad_data;
  drone_hardware_.getquaddata(quad_data);
  double current_yaw = quad_data.rpydata.z;
  std::queue<Eigen::Vector3d> zero_queue;
  for (int i = 0; i < delay_buffer_size_; ++i) {
    Eigen::Vector3d rpy(0, 0, current_yaw);
    zero_queue.push(Eigen::Vector3d::Zero());
  }
  rpy_command_buffer_.swap(zero_queue);
}

void BaseMPCControllerQuadConnector::sendControllerCommands(
    ControlType control) {
  geometry_msgs::Quaternion rpyt_msg;
  rpyt_msg.x = control(1);
  rpyt_msg.y = control(2);
  rpyt_msg.z = control(3);
  rpyt_msg.w = math::clamp(control(0), config_.min_thrust_command(),
                           config_.max_thrust_command());
  drone_hardware_.cmdrpyawratethrust(rpyt_msg);
  rpy_command_buffer_.pop();
  auto last_rpy_command = rpy_command_buffer_.back();
  // Since we are commanding yaw rate we have to integrate
  double yaw_cmd =
      control(3) * config_.dt_yaw_integration() + last_rpy_command(2);
  rpy_command_buffer_.push(Eigen::Vector3d(control(1), control(2), yaw_cmd));
  thrust_gain_estimator_.addThrustCommand(rpyt_msg.w);
}

void BaseMPCControllerQuadConnector::usePerfectTimeDiff(double time_diff) {
  config_.set_use_perfect_time_diff(true);
  config_.set_perfect_time_diff(time_diff);
}

void BaseMPCControllerQuadConnector::useSensor(
    SensorPtr<tf::StampedTransform> sensor) {
  pose_sensor_ = sensor;
}

double BaseMPCControllerQuadConnector::getTimeDiff() {
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

bool BaseMPCControllerQuadConnector::fillQuadStateAndParameters(
    Eigen::VectorXd &current_state, Eigen::VectorXd &params, double dt) {
  // Get Quad data
  parsernode::common::quaddata quad_data;
  drone_hardware_.getquaddata(quad_data);
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
  // Estimate thrust gain parameter
  thrust_gain_estimator_.addSensorData(quad_data.rpydata.x, quad_data.rpydata.y,
                                       quad_data.linacc.z);
  params.resize(1);
  params << thrust_gain_estimator_.getThrustGain();
  return true;
}
