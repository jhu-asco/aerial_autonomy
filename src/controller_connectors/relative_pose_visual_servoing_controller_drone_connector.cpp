#include "aerial_autonomy/controller_connectors/relative_pose_visual_servoing_controller_drone_connector.h"
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
  logTrackerData("relative_pose_visual_servoing_controller_drone_connector",
                 tracking_pose, object_pose_cam, quad_data);
  // giving transform in rotation-compensated quad frame
  sensor_data = std::make_tuple(getBodyFrameRotation(), tracking_pose);
  return true;
}

void RelativePoseVisualServoingControllerDroneConnector::sendControllerCommands(
    VelocityYawRate controls) {
  geometry_msgs::Vector3 velocity_cmd;
  velocity_cmd.x = controls.x;
  velocity_cmd.y = controls.y;
  velocity_cmd.z = controls.z;
  drone_hardware_.cmdvel_yaw_rate_guided(velocity_cmd, controls.yaw_rate);
}
