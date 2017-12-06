#include "aerial_autonomy/controller_hardware_connectors/rpyt_relative_pose_visual_servoing_connector.h"
#include "aerial_autonomy/log/log.h"

bool RPYTRelativePoseVisualServoingConnector::extractSensorData(
    std::tuple<tf::Transform, tf::Transform, VelocityYawRate> &sensor_data) {
  parsernode::common::quaddata quad_data;
  drone_hardware_.getquaddata(quad_data);
  tf::Transform tracking_pose;
  ///\todo Figure out what to do when the tracking pose is repeated
  if (!getTrackingTransformRotationCompensatedQuadFrame(tracking_pose)) {
    VLOG(1) << "Invalid tracking vector";
    return false;
  }
  // Estimator
  tracking_vector_estimator_.estimate(
      tracking_pose.getOrigin(),
      tf::Vector3(quad_data.linvel.x, quad_data.linvel.y, quad_data.linvel.z));
  ///\todo Check estimator health
  tf::Vector3 estimated_marker_direction =
      tracking_vector_estimator_.getMarkerDirection();
  tf::Vector3 estimated_velocity = tracking_vector_estimator_.getVelocity();
  VelocityYawRate estimated_velocity_yawrate(
      estimated_velocity.x(), estimated_velocity.y(), estimated_velocity.z(),
      quad_data.omega.z);
  auto tracking_origin = tracking_pose.getOrigin();
  double tracking_r, tracking_p, tracking_y;
  tracking_pose.getBasis().getRPY(tracking_r, tracking_p, tracking_y);
  DATA_LOG("rpyt_relative_pose_visual_servoing_connector")
      << quad_data.linvel.x << quad_data.linvel.y << quad_data.linvel.z
      << quad_data.rpydata.x << quad_data.rpydata.y << quad_data.rpydata.z
      << quad_data.omega.x << quad_data.omega.y << quad_data.omega.z
      << tracking_origin.x() << tracking_origin.y() << tracking_origin.z()
      << tracking_r << tracking_p << tracking_y << DataStream::endl;
  // Update tracking pose to use estimated marker direction instead
  // of measured direction
  tf::Transform estimated_pose(tracking_pose.getRotation(),
                               estimated_marker_direction);
  // giving transform in rotation-compensated quad frame
  sensor_data = std::make_tuple(getBodyFrameRotation(), estimated_pose,
                                estimated_velocity_yawrate);
  thrust_gain_estimator_.addSensorData(quad_data.rpydata.x, quad_data.rpydata.y,
                                       quad_data.linacc.z);
  auto rpyt_controller_config = private_reference_controller_.getRPYTConfig();
  rpyt_controller_config.set_kt(thrust_gain_estimator_.getThrustGain());
  private_reference_controller_.updateRPYTConfig(rpyt_controller_config);
  return true;
}

void RPYTRelativePoseVisualServoingConnector::sendHardwareCommands(
    RollPitchYawRateThrust controls) {
  geometry_msgs::Quaternion rpyt_msg;
  rpyt_msg.x = controls.r;
  rpyt_msg.y = controls.p;
  rpyt_msg.z = controls.y;
  rpyt_msg.w = controls.t;
  thrust_gain_estimator_.addThrustCommand(controls.t);
  drone_hardware_.cmdrpyawratethrust(rpyt_msg);
}

void RPYTRelativePoseVisualServoingConnector::setGoal(PositionYaw goal) {
  BaseClass::setGoal(goal);
  VLOG(1) << "Clearing thrust estimator buffer";
  thrust_gain_estimator_.clearBuffer();
  VLOG(1) << "Clearing initial state from kalman filter";
  tracking_vector_estimator_.resetState();
}
