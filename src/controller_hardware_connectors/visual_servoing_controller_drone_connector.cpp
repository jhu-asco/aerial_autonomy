#include "aerial_autonomy/controller_hardware_connectors/visual_servoing_controller_drone_connector.h"

PositionYaw VisualServoingControllerDroneConnector::extractSensorData() {
  parsernode::common::quaddata quad_data;
  drone_hardware_.getquaddata(quad_data);
  Position tracking_vector;
  getTrackingVectorGlobalFrame(tracking_vector);
  return PositionYaw(tracking_vector, quad_data.rpydata.z);
}

void VisualServoingControllerDroneConnector::sendHardwareCommands(
    VelocityYawRate controls) {
  geometry_msgs::Vector3 velocity_cmd;
  velocity_cmd.x = controls.x;
  velocity_cmd.y = controls.y;
  velocity_cmd.z = controls.z;
  drone_hardware_.cmdvelguided(velocity_cmd, controls.yaw_rate);
  /// \todo Gowtham Add function for commanding velocity with yaw rate
}

bool
VisualServoingControllerDroneConnector::getTrackingVectorGlobalFrame(Position& tracking_vector) {
  Position object_position_cam;
  if (!roi_to_position_converter_.getTrackingVector(object_position_cam)) {
    return false;
  }
  tf::Vector3 object_direction_cam(object_position_cam.x, object_position_cam.y,
                                   object_position_cam.z);
  // Convert from camera frame to global frame
  tf::Vector3 tracking_vector_tf = (getBodyFrameRotation() *
          (camera_transform_.getBasis() * object_direction_cam))
      .normalize();
  tracking_vector.x = tracking_vector_tf.getX();
  tracking_vector.y = tracking_vector_tf.getY();
  tracking_vector.z = tracking_vector_tf.getZ();
  return true;
}

tf::Matrix3x3 VisualServoingControllerDroneConnector::getBodyFrameRotation() {
  parsernode::common::quaddata quad_data;
  drone_hardware_.getquaddata(quad_data);
  return tf::Matrix3x3(tf::createQuaternionFromRPY(
      quad_data.rpydata.x, quad_data.rpydata.y, quad_data.rpydata.z));
}
