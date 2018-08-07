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

double BaseRelativePoseVisualServoingConnector::getViewingAngle(
    tf::Transform object_pose_cam) const {
  tf::Vector3 z_vec = camera_transform_.getBasis().getColumn(2);
  return z_vec.angle(object_pose_cam.getOrigin());
}

void BaseRelativePoseVisualServoingConnector::logTrackerData(
    string stream_name, tf::Transform tracking_pose,
    tf::Transform object_pose_cam, parsernode::common::quaddata &quad_data) {
  auto tracking_origin = tracking_pose.getOrigin();
  double tracking_r, tracking_p, tracking_y;
  tracking_pose.getBasis().getRPY(tracking_r, tracking_p, tracking_y);
  DATA_LOG(stream_name) << quad_data.linvel.x << quad_data.linvel.y
                        << quad_data.linvel.z << quad_data.rpydata.x
                        << quad_data.rpydata.y << quad_data.rpydata.z
                        << quad_data.omega.x << quad_data.omega.y
                        << quad_data.omega.z << tracking_origin.x()
                        << tracking_origin.y() << tracking_origin.z()
                        << tracking_r << tracking_p << tracking_y
                        << getViewingAngle(object_pose_cam)
                        << tracking_origin.length() << DataStream::endl;
}

void BaseRelativePoseVisualServoingConnector::logTrackerHeader(
    string stream_name) {
  DATA_HEADER(stream_name) << "vel_x"
                           << "vel_y"
                           << "vel_z"
                           << "roll"
                           << "pitch"
                           << "yaw"
                           << "omega_x"
                           << "omega_y"
                           << "omega_z"
                           << "tracking_x"
                           << "tracking_y"
                           << "tracking_z"
                           << "tracking_r"
                           << "tracking_p"
                           << "tracking_y"
                           << "Viewing_angle"
                           << "Tracking_length" << DataStream::endl;
}
