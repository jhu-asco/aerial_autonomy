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
      previous_measurements_initialized_(false),
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
