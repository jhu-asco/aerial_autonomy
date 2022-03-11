#include "aerial_autonomy/trackers/global_alvar_tracker.h"
#include "aerial_autonomy/common/conversions.h"
#include <glog/logging.h>

GlobalAlvarTracker::GlobalAlvarTracker(
            parsernode::Parser &drone_hardware,
            tf::Transform camera_transform, 
            tf::Transform tracking_offset_transform,
            double filter_gain_tracking_pose, 
            SensorPtr<std::pair<tf::StampedTransform, tf::Vector3>> odom_sensor,
            std::chrono::duration<double> timeout,
            std::string name_space)
  : AlvarTracker(timeout, name_space),
    drone_hardware_(drone_hardware),
    camera_transform_(camera_transform),
    tracking_offset_transform_(tracking_offset_transform),
    odom_sensor_(odom_sensor)
  {
    alvar_sub_ = nh_.subscribe("ar_pose_marker", 1,
                            &GlobalAlvarTracker::markerCallback, this);
  }

bool
GlobalAlvarTracker::vectorIsGlobal() {
  return true;
}

bool GlobalAlvarTracker::getTrackingVectors(
    std::unordered_map<uint32_t, tf::Transform> &pose) {
  if (!trackingIsValid()) {
    return false;
  }
  pose = target_poses_;
  return true;
}

void GlobalAlvarTracker::markerCallback(
    const ar_track_alvar_msgs::AlvarMarkers &marker_msg) {

  AlvarTracker::markerCallback(marker_msg);

  // Convert poses to world frame
  parsernode::common::quaddata quad_data;
  drone_hardware_.getquaddata(quad_data);
  tf::Transform quad_pose;
  if (odom_sensor_) {
    if (odom_sensor_->getSensorStatus() != SensorStatus::VALID) {
      LOG(WARNING) << "Pose sensor invalid!";
      return;
    }
    quad_pose = odom_sensor_->getSensorData().first;
  } else {
    quad_pose = conversions::getPose(quad_data);
  }

  // Recalculate tracking poses
  std::unordered_map<uint32_t, tf::Transform> object_poses = object_poses_;
  std::unordered_map<uint32_t, tf::Transform> target_poses;
  for (auto object : object_poses)
  {
    target_poses[object.first] =
      quad_pose * camera_transform_ * object.second * tracking_offset_transform_;
    // Filter? Separate filter per transform?
    // tracking_pose_ = filter(tracking_pose_);
  }
  target_poses_ = target_poses;
}
