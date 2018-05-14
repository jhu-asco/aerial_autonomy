#include "aerial_autonomy/controller_connectors/base_relative_pose_visual_servoing_connector.h"

tf::Transform BaseRelativePoseVisualServoingConnector::
    getTrackingTransformRotationCompensatedQuadFrame(
        tf::Transform object_pose_cam) {
  // Convert tracked frame from camera frame to UAV-centered global frame
  tf::Transform tracking_transform = getBodyFrameRotation() *
                                     camera_transform_ * object_pose_cam *
                                     tracking_offset_transform_;
  // Remove roll and pitch components of tracked frame
  double roll, pitch, yaw;
  tracking_transform.getBasis().getRPY(roll, pitch, yaw);
  tracking_transform.getBasis().setRPY(0, 0, yaw);
  return tracking_transform;
}

tf::Transform BaseRelativePoseVisualServoingConnector::getBodyFrameRotation() {
  parsernode::common::quaddata quad_data;
  drone_hardware_.getquaddata(quad_data);
  return tf::Transform(tf::createQuaternionFromRPY(quad_data.rpydata.x,
                                                   quad_data.rpydata.y,
                                                   quad_data.rpydata.z),
                       tf::Vector3(0, 0, 0));
}
