#include "aerial_autonomy/controller_hardware_connectors/visual_servoing_controller_drone_connector.h"
#include "aerial_autonomy/log/log.h"

bool VisualServoingControllerDroneConnector::extractSensorData(
    PositionYaw &sensor_data) {
  parsernode::common::quaddata quad_data;
  drone_hardware_.getquaddata(quad_data);
  Position tracking_vector;
  if (!getTrackingVectorGlobalFrame(tracking_vector)) {
    VLOG(1) << "Cannot Find tracking vector of ROI";
    return false;
  }
  DATA_LOG("visual_servoing_controller_drone_connector")
      << quad_data.linvel.x << quad_data.linvel.y << quad_data.linvel.z
      << quad_data.rpydata.x << quad_data.rpydata.y << quad_data.rpydata.z
      << quad_data.omega.x << quad_data.omega.y << quad_data.omega.z
      << DataStream::endl;
  sensor_data = PositionYaw(tracking_vector, quad_data.rpydata.z);
  return true;
}

void VisualServoingControllerDroneConnector::sendHardwareCommands(
    VelocityYawRate controls) {
  geometry_msgs::Vector3 velocity_cmd;
  velocity_cmd.x = controls.x;
  velocity_cmd.y = controls.y;
  velocity_cmd.z = controls.z;
  drone_hardware_.cmdvel_yaw_rate_guided(velocity_cmd, controls.yaw_rate);
}

bool VisualServoingControllerDroneConnector::getTrackingVectorGlobalFrame(
    Position &tracking_vector) {
  tf::Transform object_pose_cam;
  if (!tracker_.getTrackingVector(object_pose_cam)) {
    return false;
  }
  tf::Vector3 object_direction_cam = object_pose_cam.getOrigin();
  // Convert from camera frame to global frame
  tf::Vector3 tracking_vector_tf =
      (getBodyFrameRotation() *
       (camera_transform_.getBasis() * object_direction_cam));
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
