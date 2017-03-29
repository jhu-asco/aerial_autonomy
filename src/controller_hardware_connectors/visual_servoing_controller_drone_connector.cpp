#include "aerial_autonomy/controller_hardware_connectors/visual_servoing_controller_drone_connector.h"

PositionYaw VisualServoingControllerDroneConnector::extractSensorData() {
  parsernode::common::quaddata quad_data;
  drone_hardware_.getquaddata(quad_data);
  tf::Vector3 tracking_position = getTrackingVectorGlobalFrame();
  return PositionYaw(tracking_position.getX(), tracking_position.getY(),
                     tracking_position.getZ(), quad_data.rpydata.z);
}

void VisualServoingControllerDroneConnector::sendHardwareCommands(
    VelocityYawRate controls) {
  geometry_msgs::Vector3 velocity_cmd;
  velocity_cmd.x = controls.x;
  velocity_cmd.y = controls.y;
  velocity_cmd.z = controls.z;
  drone_hardware_.cmdvelguided(velocity_cmd, controls.yaw_rate);
}

tf::Vector3
VisualServoingControllerDroneConnector::getTrackingVectorGlobalFrame() {
  Position object_position_cam;
  if (!roi_to_position_converter_.getObjectPosition(object_position_cam)) {
    // \todo(Matt) handle case when cannot get object position
  }
  tf::Vector3 object_direction_cam(object_position_cam.x, object_position_cam.y,
                                   object_position_cam.z);
  // Convert from camera frame to global frame
  tf::Transform body_rpy_tf = getBodyFrameTransform();
  return (body_rpy_tf * (camera_transform_.getBasis() * object_direction_cam))
      .normalize();
}

tf::Transform VisualServoingControllerDroneConnector::getBodyFrameTransform() {
  parsernode::common::quaddata quad_data;
  drone_hardware_.getquaddata(quad_data);
  return tf::Transform(tf::createQuaternionFromRPY(
      quad_data.rpydata.x, quad_data.rpydata.y, quad_data.rpydata.z));
}
