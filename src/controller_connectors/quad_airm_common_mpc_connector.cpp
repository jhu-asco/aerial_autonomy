#include <aerial_autonomy/common/conversions.h>
#include <aerial_autonomy/common/math.h>
#include <aerial_autonomy/controller_connectors/quad_airm_common_mpc_connector.h>
#include <aerial_autonomy/log/log.h>

QuadAirmMPCCommonConnector::QuadAirmMPCCommonConnector(
    parsernode::Parser &drone_hardware,
    AbstractMPCController<StateType, ControlType> &controller,
    ThrustGainEstimator &thrust_gain_estimator, int delay_buffer_size,
    SensorPtr<tf::StampedTransform> pose_sensor,
    AbstractConstraintGeneratorPtr constraint_generator)
    : MPCControllerConnector(controller, ControllerGroup::UAV,
                             constraint_generator),
      drone_hardware_(drone_hardware), pose_sensor_(pose_sensor),
      thrust_gain_estimator_(thrust_gain_estimator),
      previous_measurement_time_(std::chrono::high_resolution_clock::now()),
      previous_measurements_initialized_(false),
      delay_buffer_size_(delay_buffer_size), private_controller_(controller),
      use_perfect_time_diff_(false), perfect_time_diff_(0.02),
      angular_exp_gain_(0.9), velocity_exp_gain_(0.9) {
  clearCommandBuffers();
}

void QuadAirmMPCCommonConnector::clearCommandBuffers() {
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

void QuadAirmMPCCommonConnector::sendControllerCommands(ControlType control) {
  geometry_msgs::Quaternion rpyt_msg;
  rpyt_msg.x = control(1);
  rpyt_msg.y = control(2);
  rpyt_msg.z = control(3);
  rpyt_msg.w = math::clamp(control(0), 40, 80);
  drone_hardware_.cmdrpyawratethrust(rpyt_msg);
  rpy_command_buffer_.pop();
  auto last_rpy_command = rpy_command_buffer_.back();
  // Since we are commanding yaw rate we have to integrate
  double yaw_cmd = control(3) * 0.02 + last_rpy_command(2);
  rpy_command_buffer_.push(Eigen::Vector3d(control(1), control(2), yaw_cmd));
  thrust_gain_estimator_.addThrustCommand(rpyt_msg.w);
}

tf::Transform
QuadAirmMPCCommonConnector::getPose(const parsernode::common::quaddata &data) {
  tf::Transform pose;
  tf::vector3MsgToTF(data.localpos, pose.getOrigin());
  pose.setRotation(tf::createQuaternionFromRPY(data.rpydata.x, data.rpydata.y,
                                               data.rpydata.z));
  return pose;
}

void QuadAirmMPCCommonConnector::usePerfectTimeDiff(double time_diff) {
  use_perfect_time_diff_ = true;
  perfect_time_diff_ = time_diff;
}

void QuadAirmMPCCommonConnector::useSensor(
    SensorPtr<tf::StampedTransform> sensor) {
  pose_sensor_ = sensor;
}

Eigen::Vector3d
QuadAirmMPCCommonConnector::omegaToRpyDot(const Eigen::Vector3d &omega,
                                          const Eigen::Vector3d &rpy,
                                          const double max_pitch) {
  Eigen::Matrix3d Mrpy;
  double pitch = rpy[1];
  double abs_pitch = std::abs(pitch);
  if (abs_pitch > max_pitch) {
    LOG(WARNING) << "Pitch close to pi/2 which is a singularity";
    pitch = std::copysign(max_pitch, pitch);
  }
  double s_roll = sin(rpy[0]), c_roll = cos(rpy[0]), t_pitch = tan(pitch);
  double sec_pitch = sqrt(1 + t_pitch * t_pitch);
  Mrpy << 1, s_roll * t_pitch, c_roll * t_pitch, 0, c_roll, -s_roll, 0,
      s_roll * sec_pitch, c_roll * sec_pitch;
  return Mrpy * omega;
}
