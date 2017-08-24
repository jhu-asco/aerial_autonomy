#include "aerial_autonomy/controller_hardware_connectors/visual_servoing_controller_arm_connector.h"
#include "aerial_autonomy/common/conversions.h"

bool VisualServoingControllerArmConnector::extractSensorData(
    std::tuple<tf::Transform, tf::Transform> &sensor_data) {
  tf::Transform tracking_pose;
  if (!getTrackingPoseArmFrame(tracking_pose)) {
    VLOG(1) << "Cannot find tracking pose";
    return false;
  }

  tf::Transform arm_pose_tf;
  conversions::transformMatrix4dToTf(arm_hardware_.getEndEffectorTransform(),
                                     arm_pose_tf);

  sensor_data = std::make_tuple(arm_pose_tf, tracking_pose);

  return true;
}

void VisualServoingControllerArmConnector::sendHardwareCommands(
    tf::Transform pose) {
  Eigen::Affine3d pose_eig;
  tf::transformTFToEigen(pose, pose_eig);
  if (!arm_hardware_.setEndEffectorPose(pose_eig.matrix())) {
    LOG_EVERY_N(WARNING, 50) << "End effector not in workspace";
  }
}

bool VisualServoingControllerArmConnector::getTrackingPoseArmFrame(
    tf::Transform &tracking_pose) {
  Position object_position_cam;
  if (!tracker_.getTrackingVector(object_position_cam)) {
    return false;
  }

  // Get drone orientation
  parsernode::common::quaddata quad_data;
  drone_hardware_.getquaddata(quad_data);
  tf::Transform body_rotation;
  conversions::transformRPYToTf(quad_data.rpydata.x, quad_data.rpydata.y,
                                quad_data.rpydata.z, body_rotation);

  // Compute object transform in drone frame
  tf::Transform object_rotation_global_frame;
  // Since rotation is ambiguous for 2D tracker, assume gravity aligned frame
  // with same yaw as drone
  conversions::transformRPYToTf(0, 0, quad_data.rpydata.z,
                                object_rotation_global_frame);
  tf::Transform object_rotation_body_frame =
      body_rotation.inverse() * object_rotation_global_frame;

  tf::Vector3 object_direction_cam(object_position_cam.x, object_position_cam.y,
                                   object_position_cam.z);
  tf::Vector3 object_position_body_frame =
      camera_transform_ * object_direction_cam;

  tf::Transform object_tf_body_frame(object_rotation_body_frame.getRotation(),
                                     object_position_body_frame);

  // Compute object transform in arm frame
  tracking_pose = arm_transform_.inverse() * object_tf_body_frame;

  return true;
}
