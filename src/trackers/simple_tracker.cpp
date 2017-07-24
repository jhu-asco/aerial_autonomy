#include "aerial_autonomy/trackers/simple_tracker.h"
#include <glog/logging.h>

SimpleTracker::SimpleTracker(parsernode::Parser &drone_hardware,
                             tf::Transform camera_transform)
    : drone_hardware_(drone_hardware), camera_transform_(camera_transform) {
  tracking_valid_ = true;
}

bool SimpleTracker::getTrackingVector(Position &p) {
  if (!trackingIsValid()) {
    return false;
  }
  parsernode::common::quaddata uav_data;
  drone_hardware_.getquaddata(uav_data);
  tf::Transform quad_tf_global(
      tf::createQuaternionFromRPY(uav_data.rpydata.x, uav_data.rpydata.y,
                                  uav_data.rpydata.z),
      tf::Vector3(uav_data.localpos.x, uav_data.localpos.y,
                  uav_data.localpos.z));
  tf::Vector3 target_position_global(target_position_.x, target_position_.y,
                                     target_position_.z);
  tf::Vector3 target_position_camera = camera_transform_.inverse() *
                                       quad_tf_global.inverse() *
                                       target_position_global;
  p = Position(target_position_camera.getX(), target_position_camera.getY(),
               target_position_camera.getZ());
  return true;
}

void SimpleTracker::setTargetPositionGlobalFrame(Position p) {
  target_position_ = p;
}

bool SimpleTracker::trackingIsValid() { return tracking_valid_; }

void SimpleTracker::setTrackingIsValid(bool is_valid) {
  tracking_valid_ = is_valid;
}

tf::Transform SimpleTracker::cameraTransform() { return camera_transform_; }
