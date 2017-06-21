#include "aerial_autonomy/controller_hardware_connectors/visual_servoing_controller_arm_connector.h"

bool VisualServoingControllerArmConnector::extractSensorData(
    std::tuple<Position, Position> &sensor_data) {
  Position tracking_position;
  if (!getTrackingVectorArmFrame(tracking_position)) {
    VLOG(1) << "Cannot find tracking vector";
    return false;
  }

  Eigen::Matrix4d arm_tf = arm_hardware_.getEndEffectorTransform();
  Position arm_position(arm_tf(0, 3), arm_tf(1, 3), arm_tf(2, 3));

  sensor_data = std::make_tuple(arm_position, tracking_position);

  return true;
}

void VisualServoingControllerArmConnector::sendHardwareCommands(
    Position controls) {
  Eigen::Matrix4d pose;
  pose.setIdentity();
  pose(0, 3) = controls.x;
  pose(1, 3) = controls.y;
  pose(2, 3) = controls.z;
  arm_hardware_.setEndEffectorPose(pose);
}

bool VisualServoingControllerArmConnector::getTrackingVectorArmFrame(
    Position &tracking_vector) {
  Position object_position_cam;
  if (!tracker_.getTrackingVector(object_position_cam)) {
    return false;
  }
  tf::Vector3 object_direction_cam(object_position_cam.x, object_position_cam.y,
                                   object_position_cam.z);
  // TODO: Convert from camera frame to arm frame
  tf::Vector3 tracking_vector_tf =
      arm_transform_.inverse() * camera_transform_ * object_direction_cam;
  tracking_vector.x = tracking_vector_tf.getX();
  tracking_vector.y = tracking_vector_tf.getY();
  tracking_vector.z = tracking_vector_tf.getZ();
  return true;
}
