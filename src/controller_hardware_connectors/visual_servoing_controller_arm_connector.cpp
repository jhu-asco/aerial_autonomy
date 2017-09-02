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
  tf::Transform object_pose_cam;
  if (!tracker_.getTrackingVector(object_pose_cam)) {
    return false;
  }
  /**
  * \todo Matt
  *
  *  This will break RoiToPositionConverter since the object rotation will
  * always be identity.
  *  The Proper way to do this will require two types of trackers: 1. Position
  * tracker, 2. Pose tracker.
  *  We can have two different versions of the connector (one for each tracker
  * type) subclassed from a common base.
  *  The subclasses will override this function
  */
  // Compute object transform in quad frame
  tf::Transform object_tf_body_frame = camera_transform_ * object_pose_cam;

  // Compute object transform in arm frame
  tracking_pose = arm_transform_.inverse() * object_tf_body_frame;

  return true;
}
