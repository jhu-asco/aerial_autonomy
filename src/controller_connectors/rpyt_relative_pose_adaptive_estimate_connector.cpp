#include "aerial_autonomy/controller_connectors/rpyt_relative_pose_adaptive_estimate_connector.h"
#include "aerial_autonomy/log/log.h"

bool RPYTRelativePoseAdaptiveEstimateConnector::extractSensorData(
    std::tuple<double, double, ParticleStateYaw> &sensor_data) {
  ParticleStateYaw curr_state;
  // Get quad data from parser
  parsernode::common::quaddata quad_data;
  drone_hardware_.getquaddata(quad_data);
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
    curr_state.yaw = rpy(2);
  } else {
    // Pose
    curr_state.p = Position(quad_data.localpos.x, quad_data.localpos.y,
                            quad_data.localpos.z);
    // Linear velocity
    curr_state.v =
        Velocity(quad_data.linvel.x, quad_data.linvel.y, quad_data.linvel.z);
    // Yaw
    curr_state.yaw = quad_data.rpydata.z;
  }
  if (use_perfect_time_diff_) {
    time_since_init_ += perfect_time_diff_;
  } else {
    time_since_init_ = std::chrono::duration<double>(
                           std::chrono::high_resolution_clock::now() - t_init_)
                           .count();
  }
  sensor_data = std::make_tuple(mhat_, time_since_init_, curr_state);
  //Add to thrust gain estimator for comparison purposes
  tf::Vector3 body_acc(quad_data.linacc.x, quad_data.linacc.y,
                       quad_data.linacc.z);
  thrust_gain_estimator_.addSensorData(quad_data.rpydata.x, quad_data.rpydata.y,
                                       body_acc);
  return true;
}

void RPYTRelativePoseAdaptiveEstimateConnector::sendControllerCommands(
    RollPitchYawThrustAdaptive controls) {
  // Calculate dt and mhat
  double dt;
  if (use_perfect_time_diff_) {
    dt = perfect_time_diff_;
  }else {
    std::chrono::time_point<std::chrono::high_resolution_clock> current_time =
        std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> dt_duration =
        std::chrono::duration_cast<std::chrono::duration<double>>(current_time -
                                                                  previous_time_);
    previous_time_ = current_time;
    dt = dt_duration.count();
  }
  mhat_ += controls.dm * dt;
  if (mhat_ < min_m_) {
    mhat_ = min_m_;
  }
  // Send commands
  geometry_msgs::Quaternion rpyt_msg;
  rpyt_msg.x = controls.r;
  rpyt_msg.y = controls.p;
  rpyt_msg.z = controls.y;
  rpyt_msg.w = controls.t;
  drone_hardware_.cmdrpyawratethrust(rpyt_msg);
  //Add to thrust gain estimator for comparison purposes
  thrust_gain_estimator_.addThrustCommand(controls.t);
}

void RPYTRelativePoseAdaptiveEstimateConnector::setGoal(
    ReferenceTrajectoryPtr<ParticleStateYaw, Snap> goal) {
  //This may need a t_init_ to be set too. 
  //previous_time_ = std::chrono::high_resolution_clock::now();
  BaseClass::setGoal(goal);
}

void RPYTRelativePoseAdaptiveEstimateConnector::initialize() {
  t_init_ = std::chrono::high_resolution_clock::now();
  previous_time_ = t_init_;
  time_since_init_ = 0;
}
