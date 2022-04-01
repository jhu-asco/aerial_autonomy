#include "aerial_autonomy/trackers/global_object_tracker.h"
#include "aerial_autonomy/common/conversions.h"
#include <glog/logging.h>

GlobalObjectTracker::GlobalObjectTracker(
            parsernode::Parser &drone_hardware,
            tf::Transform camera_transform,
            tf::Transform tracking_offset_transform,
            double filter_gain_tracking_pose,
            SensorPtr<std::pair<tf::StampedTransform, tf::Vector3>> odom_sensor,
            std::chrono::duration<double> timeout,
            std::string name_space)
  : ObjectTracker(timeout, name_space),
    drone_hardware_(drone_hardware),
    camera_transform_(camera_transform),
    tracking_offset_transform_(tracking_offset_transform),
    odom_sensor_(odom_sensor),
    filter_gain_tracking_pose_(filter_gain_tracking_pose)
  {
    detection_sub_ = nh_.subscribe("object_detections", 1,
                            &GlobalObjectTracker::detectionCallback, this);
  }

tf::Transform GlobalObjectTracker::filter(uint32_t id, tf::Transform input) {
  boost::mutex::scoped_lock lock(filter_mutex_);
  // If there isn't a filter for that key, add one
  if (tracking_pose_filters_.find(id) == tracking_pose_filters_.end())
  {
    tracking_pose_filters_.insert({id, ExponentialFilter<PositionYaw>(filter_gain_tracking_pose_)});
  }

  PositionYaw tracking_position_yaw;
  conversions::tfToPositionYaw(tracking_position_yaw, input);
  PositionYaw filtered_position_yaw =
      tracking_pose_filters_.at(id).addAndFilter(tracking_position_yaw);
  tf::Transform filtered_pose;
  conversions::positionYawToTf(filtered_position_yaw, filtered_pose);
  return filtered_pose;
}

void GlobalObjectTracker::resetFilters() {
  boost::mutex::scoped_lock lock(filter_mutex_);
  for (auto filter : tracking_pose_filters_)
  {
    (filter.second).reset();
  }
}

bool
GlobalObjectTracker::vectorIsGlobal() {
  return true;
}

bool GlobalObjectTracker::getTrackingVectors(
    std::unordered_map<uint32_t, tf::Transform> &pose) {
  if (!trackingIsValid()) {
    return false;
  }
  pose = target_poses_;
  return true;
}

void GlobalObjectTracker::detectionCallback(
    const vision_msgs::Detection3DArray &detect_msg) {

  // Return if there are not any detections
  if (detect_msg.detections.size() == 0)
    return;

  ObjectTracker::detectionCallback(detect_msg);

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

    // Filter (and removes rp)
    target_poses[object.first] = filter(object.first, target_poses[object.first]);
  }
  target_poses_ = target_poses;
}
