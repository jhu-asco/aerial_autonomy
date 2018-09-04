#include <aerial_autonomy/common/conversions.h>
#include <aerial_autonomy/common/math.h>
#include <aerial_autonomy/controller_connectors/base_mpc_controller_quad_connector.h>
#include <aerial_autonomy/log/log.h>

BaseMPCControllerQuadConnector::BaseMPCControllerQuadConnector(
    parsernode::Parser &drone_hardware,
    AbstractMPCController<StateType, ControlType> &controller,
    ThrustGainEstimator &thrust_gain_estimator, int delay_buffer_size,
    MPCConnectorConfig config,
    SensorPtr<std::pair<tf::StampedTransform, tf::Vector3>> odom_sensor,
    AbstractConstraintGeneratorPtr constraint_generator)
    : MPCControllerConnector(controller, ControllerGroup::UAV,
                             constraint_generator),
      drone_hardware_(drone_hardware), odom_sensor_(odom_sensor),
      thrust_gain_estimator_(thrust_gain_estimator),
      rpydot_filter_(config.rpydot_gain()),
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
  Eigen::Vector2d roll_pitch_bias = thrust_gain_estimator_.getRollPitchBias();
  rpyt_msg.x = control(1) - roll_pitch_bias[0];
  rpyt_msg.y = control(2) - roll_pitch_bias[1];
  rpyt_msg.z = control(3);
  rpyt_msg.w = math::clamp(control(0), config_.min_thrust_command(),
                           config_.max_thrust_command());
  VLOG_EVERY_N(1, 20) << "Control: " << rpyt_msg.w << ", " << rpyt_msg.x << ", "
                      << rpyt_msg.y << ", " << rpyt_msg.z;
  drone_hardware_.cmdrpyawratethrust(rpyt_msg);
  rpy_command_buffer_.pop();
  auto last_rpy_command = rpy_command_buffer_.back();
  // Since we are commanding yaw rate we have to integrate
  double yaw_cmd =
      control(3) * config_.dt_yaw_integration() + last_rpy_command(2);
  yaw_cmd = math::angleWrap(yaw_cmd);
  rpy_command_buffer_.push(Eigen::Vector3d(control(1), control(2), yaw_cmd));
  thrust_gain_estimator_.addThrustCommand(rpyt_msg.w);
}

void BaseMPCControllerQuadConnector::usePerfectTimeDiff(double time_diff) {
  config_.set_use_perfect_time_diff(true);
  config_.set_perfect_time_diff(time_diff);
}

void BaseMPCControllerQuadConnector::useSensor(
    SensorPtr<std::pair<tf::StampedTransform, tf::Vector3>> sensor) {
  odom_sensor_ = sensor;
}

bool BaseMPCControllerQuadConnector::fillQuadStateAndParameters(
    Eigen::VectorXd &current_state, Eigen::VectorXd &params) {
  // Get Quad data
  parsernode::common::quaddata quad_data;
  drone_hardware_.getquaddata(quad_data);
  tf::Transform quad_pose;
  Eigen::Vector3d velocity;
  if (odom_sensor_) {
    if (odom_sensor_->getSensorStatus() != SensorStatus::VALID) {
      LOG(WARNING) << "Sensor invalid";
      return false;
    }
    auto quad_pose_velocity_pair = odom_sensor_->getSensorData();
    quad_pose = quad_pose_velocity_pair.first;
    velocity = Eigen::Vector3d(quad_pose_velocity_pair.second.x(),
                               quad_pose_velocity_pair.second.y(),
                               quad_pose_velocity_pair.second.z());
  } else {
    quad_pose = conversions::getPose(quad_data);
    velocity = Eigen::Vector3d(quad_data.linvel.x, quad_data.linvel.y,
                               quad_data.linvel.z);
  }
  // Position
  const auto &quad_position = quad_pose.getOrigin();
  Eigen::Vector3d p(quad_position.x(), quad_position.y(), quad_position.z());
  Eigen::Vector3d rpy;
  Eigen::Vector3d omega(quad_data.omega.x, quad_data.omega.y,
                        quad_data.omega.z);
  Eigen::Vector2d roll_pitch_bias = thrust_gain_estimator_.getRollPitchBias();
  // Euler angles
  if (odom_sensor_) {
    rpy = conversions::transformTfToRPY(quad_pose);
    rpy[0] = quad_data.rpydata.x + roll_pitch_bias[0];
    rpy[1] = quad_data.rpydata.y + roll_pitch_bias[1];
  } else {
    rpy = Eigen::Vector3d(quad_data.rpydata.x, quad_data.rpydata.y,
                          quad_data.rpydata.z);
    rpy[0] = quad_data.rpydata.x + roll_pitch_bias[0];
    rpy[1] = quad_data.rpydata.y + roll_pitch_bias[1];
  }
  Eigen::Vector3d filtered_rpydot =
      rpydot_filter_.addAndFilter(conversions::omegaToRpyDot(omega, rpy));
  // Fill state
  current_state.segment<3>(0) = p;
  current_state.segment<3>(3) = rpy;
  current_state.segment<3>(6) = velocity;
  current_state.segment<3>(9) = filtered_rpydot;
  current_state.segment<3>(12) = rpy_command_buffer_.front();
  // Estimate thrust gain parameter
  tf::Vector3 body_acc(quad_data.linacc.x, quad_data.linacc.y,
                       quad_data.linacc.z);
  thrust_gain_estimator_.addSensorData(quad_data.rpydata.x, quad_data.rpydata.y,
                                       body_acc);
  params.resize(1);
  params << thrust_gain_estimator_.getThrustGain();
  return true;
}
