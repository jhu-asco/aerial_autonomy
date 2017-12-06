#include "aerial_autonomy/controller_hardware_connectors/relative_pose_visual_servoing_controller_drone_connector.h"
#include "aerial_autonomy/log/log.h"

bool RelativePoseVisualServoingControllerDroneConnector::extractSensorData(
    std::tuple<tf::Transform, tf::Transform> &sensor_data) {
  parsernode::common::quaddata quad_data;
  drone_hardware_.getquaddata(quad_data);
  tf::Transform object_pose_cam;
  if (!tracker_.getTrackingVector(object_pose_cam)) {
    VLOG(1) << "Invalid tracking vector";
    return false;
  }
  tf::Transform tracking_pose =
      getTrackingTransformRotationCompensatedQuadFrame(object_pose_cam);
  DATA_LOG("relative_pose_visual_servoing_controller_drone_connector")
      << quad_data.linvel.x << quad_data.linvel.y << quad_data.linvel.z
      << quad_data.rpydata.x << quad_data.rpydata.y << quad_data.rpydata.z
      << quad_data.omega.x << quad_data.omega.y << quad_data.omega.z
      << DataStream::endl;
  // giving transform in rotation-compensated quad frame
  sensor_data = std::make_tuple(getBodyFrameRotation(), tracking_pose);
  return true;
}

void RelativePoseVisualServoingControllerDroneConnector::sendHardwareCommands(
    VelocityYawRate controls) {
  geometry_msgs::Vector3 velocity_cmd;
  velocity_cmd.x = controls.x;
  velocity_cmd.y = controls.y;
  velocity_cmd.z = controls.z;
  drone_hardware_.cmdvel_yaw_rate_guided(velocity_cmd, controls.yaw_rate);
}
