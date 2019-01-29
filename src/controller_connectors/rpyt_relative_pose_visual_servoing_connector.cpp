#include "aerial_autonomy/controller_connectors/rpyt_relative_pose_visual_servoing_connector.h"
#include "aerial_autonomy/log/log.h"

bool RPYTRelativePoseVisualServoingConnector::extractSensorData(
    std::tuple<tf::Transform, tf::Transform, VelocityYawRate> &sensor_data) {
  parsernode::common::quaddata quad_data;
  drone_hardware_.getquaddata(quad_data);
  tf::Transform object_pose_cam;
  ///\todo Figure out what to do when the tracking pose is repeated
  if (!tracker_.getTrackingVector(object_pose_cam)) {
    VLOG(1) << "Invalid tracking vector";
    return false;
  }
  tf::Transform body_frame_rotation = getBodyFrameRotation();
  tf::Transform tracking_pose =
      getTrackingTransformRotationCompensatedQuadFrame(object_pose_cam,
                                                       body_frame_rotation);
  logTrackerData("rpyt_relative_pose_visual_servoing_connector", tracking_pose,
                 object_pose_cam, quad_data);
  // giving transform in rotation-compensated quad frame
  sensor_data =
      std::make_tuple(body_frame_rotation, tracking_pose,
                      VelocityYawRate(quad_data.linvel.x, quad_data.linvel.y,
                                      quad_data.linvel.z, quad_data.omega.z));
  tf::Vector3 body_acc(quad_data.linacc.x, quad_data.linacc.y,
                       quad_data.linacc.z);
  Eigen::Vector3d body_acc_eig(quad_data.linacc.x, quad_data.linacc.y,
                               quad_data.linacc.z);
  thrust_gain_estimator_.addSensorData(quad_data.rpydata.x, quad_data.rpydata.y,
                                       body_acc);
  acceleration_bias_estimator_.addSensorData(quad_data.rpydata.x,
                                             quad_data.rpydata.y, body_acc_eig);
  auto rpyt_controller_config = private_reference_controller_.getRPYTConfig();
  rpyt_controller_config.set_kt(thrust_gain_estimator_.getThrustGain());
  private_reference_controller_.updateRPYTConfig(rpyt_controller_config);
  return true;
}

void RPYTRelativePoseVisualServoingConnector::sendControllerCommands(
    RollPitchYawRateThrust controls) {
  geometry_msgs::Quaternion rpyt_msg;
  Eigen::Vector2d roll_pitch_bias = thrust_gain_estimator_.getRollPitchBias();
  rpyt_msg.x = controls.r - roll_pitch_bias[0];
  rpyt_msg.y = controls.p - roll_pitch_bias[1];
  rpyt_msg.z = controls.y;
  rpyt_msg.w = controls.t;
  thrust_gain_estimator_.addThrustCommand(controls.t);
  acceleration_bias_estimator_.addAccelerationCommand(
      controls.t * thrust_gain_estimator_.getThrustGain());
  drone_hardware_.cmdrpyawratethrust(rpyt_msg);
}

void RPYTRelativePoseVisualServoingConnector::setGoal(PositionYaw goal) {
  BaseClass::setGoal(goal);
  VLOG(1) << "Clearing thrust estimator buffer";
  thrust_gain_estimator_.clearBuffer();
}
