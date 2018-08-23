#include "aerial_autonomy/controller_connectors/rpyt_based_reference_connector.h"

template <class StateT, class ControlT>
bool RPYTBasedReferenceConnector<StateT, ControlT>::extractSensorData(
    std::tuple<double, double, Velocity, PositionYaw> &sensor_data) {
  parsernode::common::quaddata data;
  drone_hardware_.getquaddata(data);
  PositionYaw position_yaw;
  Velocity velocity;
  if (pose_sensor_) {
    if (pose_sensor_->getSensorStatus() != SensorStatus::VALID) {
      LOG(WARNING) << "Sensor invalid";
      return false;
    }
    tf::Transform quad_pose = pose_sensor_->getSensorData();
    conversions::tfToPositionYaw(position_yaw, quad_pose);
    // Get Velocity
    tf::Vector3 sensor_velocity = getVelocity(quad_pose.getOrigin());
    velocity =
        Velocity(sensor_velocity.x(), sensor_velocity.y(), sensor_velocity.z());
  } else {
    position_yaw = PositionYaw(data.localpos.x, data.localpos.y,
                               data.localpos.z, data.rpydata.z);
    velocity = Velocity(data.linvel.x, data.linvel.y, data.linvel.z);
  }
  double time_since_init =
      std::chrono::duration<double>(std::chrono::high_resolution_clock::now() -
                                    t_init_)
          .count();
  sensor_data =
      std::make_tuple(time_since_init, thrust_gain_estimator_.getThrustGain(),
                      velocity, position_yaw);
  thrust_gain_estimator_.addSensorData(data.rpydata.x, data.rpydata.y,
                                       data.linacc.z);
  return true;
}

template <class StateT, class ControlT>
tf::Vector3 RPYTBasedReferenceConnector<StateT, ControlT>::getVelocity(
    tf::Vector3 current_position) {
  double dt = getTimeDiff();
  tf::Vector3 velocity(0, 0, 0);
  if (previous_measurement_initialized_) {
    velocity = (current_position - previous_position_) / dt;
    previous_measurement_initialized_ = false;
  }
  return velocity;
}

template <class StateT, class ControlT>
double RPYTBasedReferenceConnector<StateT, ControlT>::getTimeDiff() {
  // Timing logic
  auto current_time = std::chrono::high_resolution_clock::now();
  double dt =
      std::chrono::duration<double>(current_time - previous_measurement_time_)
          .count();
  if (use_perfect_time_diff_) {
    dt = perfect_time_diff_;
  }
  previous_measurement_time_ = current_time;
  if (dt < 1e-4) {
    LOG(WARNING) << "Time diff cannot be smaller than 1e-4";
    dt = 1e-4;
  }
  return dt;
}

template <class StateT, class ControlT>
void RPYTBasedReferenceConnector<StateT, ControlT>::sendControllerCommands(
    RollPitchYawRateThrust controls) {
  geometry_msgs::Quaternion rpyt_msg;
  rpyt_msg.x = controls.r;
  rpyt_msg.y = controls.p;
  rpyt_msg.z = controls.y;
  rpyt_msg.w = controls.t;
  thrust_gain_estimator_.addThrustCommand(controls.t);
  drone_hardware_.cmdrpyawratethrust(rpyt_msg);
}

template <class StateT, class ControlT>
void RPYTBasedReferenceConnector<StateT, ControlT>::setGoal(PositionYaw goal) {
  BaseClass::setGoal(goal);
  VLOG(1) << "Clearing thrust estimator buffer";
  thrust_gain_estimator_.clearBuffer();
}

template <class StateT, class ControlT>
void RPYTBasedReferenceConnector<StateT, ControlT>::initialize() {
  t_init_ = std::chrono::high_resolution_clock::now();
  velocity_filter_.reset();
  previous_measurement_initialized_ = false;
}
