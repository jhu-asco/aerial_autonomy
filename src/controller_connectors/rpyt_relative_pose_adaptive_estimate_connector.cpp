#include "aerial_autonomy/controller_connectors/rpyt_relative_pose_adaptive_estimate_connector.h"
#include "aerial_autonomy/log/log.h"

/*
* @brief
*/
bool RPYTRelativePoseAdaptiveEstimateConnector::extractSensorData(
    std::tuple<double, double, ParticleState> &sensor_data) {
  ParticleState curr_state;
  double curr_yaw;
  if (odom_sensor_) {
    // Use outside sensor (if valid)
    if (odom_sensor_->getSensorStatus() != SensorStatus::VALID) {
      LOG(WARNING) << "Pose sensor invalid!";
      return false;
    }
    auto quad_pose_velocity_pair = odom_sensor_->getSensorData();
    // Pose
    tf::Vector3 pos = quad_pose_velocity_pair.first.getOrigin();
    curr_state.p = Position(pos.getX(), pos.getY(), pos.getZ());
    // Linear velocity
    tf::Vector3 vel = quad_pose_velocity_pair.second;
    curr_state.v = Velocity(vel.getX(), vel.getY(), vel.getZ());
    // Yaw
    Eigen::Vector3d rpy =
        conversions::transformTfToRPY(quad_pose_velocity_pair.first);
    curr_yaw = rpy(2);
  } else {
    // Get quad data from parser
    parsernode::common::quaddata quad_data;
    drone_hardware_.getquaddata(quad_data);
    // Pose
    curr_state.p = Position(quad_data.localpos.x, quad_data.localpos.y,
                            quad_data.localpos.z);
    // Linear velocity
    curr_state.v =
        Velocity(quad_data.linvel.x, quad_data.linvel.y, quad_data.linvel.z);
    // Yaw
    curr_yaw = quad_data.rpydata.z;
  }
  sensor_data = std::make_tuple(mhat, curr_yaw, curr_state);
  return true;
}

void RPYTRelativePoseAdaptiveEstimateConnector::sendControllerCommands(
    RollPitchYawThrustAdaptive controls) {
  // Calculate dt and mhat
  std::chrono::time_point<std::chrono::high_resolution_clock> current_time =
      std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> dt_duration =
      std::chrono::duration_cast<std::chrono::duration<double>>(current_time -
                                                                previous_time_);
  double dt = dt_duration.count();
  previous_time_ = current_time;
  mhat += controls.dm * dt;
  if (mhat < min_m_) {
    mhat = min_m_;
  }
  // Send commands
  geometry_msgs::Quaternion rpyt_msg;
  rpyt_msg.x = controls.r;
  rpyt_msg.y = controls.p;
  rpyt_msg.z = controls.y;
  rpyt_msg.w = controls.t;
  drone_hardware_.cmdrpyawratethrust(rpyt_msg);
}

void RPYTRelativePoseAdaptiveEstimateConnector::setGoal(
    std::pair<ReferenceTrajectoryPtr<ParticleState, Snap>, double> goal) {
  BaseClass::setGoal(goal);
}
