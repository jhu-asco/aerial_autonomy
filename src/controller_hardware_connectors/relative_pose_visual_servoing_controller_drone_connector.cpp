#include "aerial_autonomy/controller_hardware_connectors/relative_pose_visual_servoing_controller_drone_connector.h"
#include "aerial_autonomy/log/log.h"

bool RelativePoseVisualServoingControllerDroneConnector::extractSensorData(
    std::tuple<tf::Transform, tf::Transform> &sensor_data) {
  parsernode::common::quaddata quad_data;
  drone_hardware_.getquaddata(quad_data);
  tf::Transform tracking_pose;
  if (!getTrackingTransformRotationCompensatedQuadFrame(tracking_pose)) {
    VLOG(1) << "Invalid tracking vector";
    return false;
  }
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

bool RelativePoseVisualServoingControllerDroneConnector::
    getTrackingTransformRotationCompensatedQuadFrame(
        tf::Transform &tracking_transform) {
  tf::Transform object_pose_cam;
  if (!tracker_.getTrackingVector(object_pose_cam)) {
    return false;
  }
  // Convert from camera frame to global frame
  tracking_transform =
      getBodyFrameRotation() * camera_transform_ * object_pose_cam;
  // Remove roll and pitch components of tracked frame
  double roll, pitch, yaw;
  tracking_transform.getBasis().getRPY(roll, pitch, yaw);
  tracking_transform.getBasis().setRPY(0, 0, yaw);
  return true;
}

tf::Transform
RelativePoseVisualServoingControllerDroneConnector::getBodyFrameRotation() {
  parsernode::common::quaddata quad_data;
  drone_hardware_.getquaddata(quad_data);
  return tf::Transform(tf::createQuaternionFromRPY(quad_data.rpydata.x,
                                                   quad_data.rpydata.y,
                                                   quad_data.rpydata.z),
                       tf::Vector3(0, 0, 0));
}
