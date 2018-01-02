#include "aerial_autonomy/controller_hardware_connectors/rpyt_relative_pose_visual_servoing_connector.h"
#include "aerial_autonomy/log/log.h"

bool RPYTRelativePoseVisualServoingConnector::extractSensorData(
    std::tuple<tf::Transform, tf::Transform, VelocityYawRate> &sensor_data) {
  parsernode::common::quaddata quad_data;
  drone_hardware_.getquaddata(quad_data);
  VelocityYawRate current_velocity_yawrate(
      quad_data.linvel.x, quad_data.linvel.y, quad_data.linvel.z,
      quad_data.omega.z);
  tf::Transform object_pose_cam;
  ///\todo Figure out what to do when the tracking pose is repeated
  if (!tracker_.getTrackingVector(object_pose_cam)) {
    VLOG(1) << "Invalid tracking vector";
    return false;
  }
  tf::Transform tracking_pose =
      getTrackingTransformRotationCompensatedQuadFrame(object_pose_cam);
  auto tracking_origin = tracking_pose.getOrigin();
  double tracking_r, tracking_p, tracking_y;
  tracking_pose.getBasis().getRPY(tracking_r, tracking_p, tracking_y);
  DATA_LOG("rpyt_relative_pose_visual_servoing_connector")
      << quad_data.linvel.x << quad_data.linvel.y << quad_data.linvel.z
      << quad_data.rpydata.x << quad_data.rpydata.y << quad_data.rpydata.z
      << quad_data.omega.x << quad_data.omega.y << quad_data.omega.z
      << tracking_origin.x() << tracking_origin.y() << tracking_origin.z()
      << tracking_r << tracking_p << tracking_y
      << getViewingAngle(object_pose_cam) << tracking_origin.length()
      << DataStream::endl;
  // giving transform in rotation-compensated quad frame
  sensor_data =
      std::make_tuple(getBodyFrameRotation(), tracking_pose,
                      VelocityYawRate(quad_data.linvel.x, quad_data.linvel.y,
                                      quad_data.linvel.z, quad_data.omega.z));
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
}

double RPYTRelativePoseVisualServoingConnector::getViewingAngle(
    tf::Transform object_pose_cam) const {
  tf::Vector3 z_vec = camera_transform_.getBasis().getColumn(2);
  return z_vec.angle(object_pose_cam.getOrigin());
}
