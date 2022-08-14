#include "aerial_autonomy/trackers/object_tracker.h"

#include <glog/logging.h>

bool ObjectTracker::getTrackingVectors(
    std::unordered_map<uint32_t, tf::Transform> &pose) {
  if (!trackingIsValid()) {
    return false;
  }
  pose = object_poses_;
  return true;
}

bool ObjectTracker::trackingIsValid() {
  std::unordered_map<uint32_t, tf::Transform> object_poses = object_poses_;
  std::unordered_map<uint32_t, ros::Time> last_valid_times = last_valid_times_;
  std::vector<uint32_t> removed_objects;
  bool valid = trackingIsValidAndRemoveInvalidPoses(object_poses, last_valid_times, removed_objects);

  object_poses_ = object_poses;
  last_valid_times_ = last_valid_times;
  
  return valid;
}

bool ObjectTracker::trackingIsValidAndRemoveInvalidPoses(
  std::unordered_map<uint32_t, tf::Transform> &object_poses,
  std::unordered_map<uint32_t, ros::Time> &last_valid_times,
  std::vector<uint32_t> &removed_objects)
{
  ros::Time current_time = ros::Time::now();
  std::vector<uint32_t> objects_to_remove;
  for (auto object : object_poses)
  {
    // Check if invalid
    if ((current_time - last_valid_times[object.first]).toSec() > timeout_.count())
    {
      objects_to_remove.emplace_back(object.first);
    }
  }

  for (auto key : objects_to_remove)
  {
    // Remove invalid objects
    object_poses.erase(key);
    last_valid_times.erase(key);
  }

  removed_objects = objects_to_remove;

  bool valid = (object_poses.size() > 0);
  if (!valid) {
    VLOG_EVERY_N(2, 100) << "Object detection has not been updated for "
                         << timeout_.count() << " seconds";
  }
  
  return valid;
}

void ObjectTracker::detectionCallback(
    const vision_msgs::Detection3DArray &detect_msg) 
{
  if (detect_msg.detections.size() == 0)
    return;

  getObjectPoses(detect_msg);
}

std::unordered_map<uint32_t, tf::Transform> ObjectTracker::getObjectPoses(
    const vision_msgs::Detection3DArray &detect_msg)
{
  ros::Time current_time = ros::Time::now();
  last_tracking_time_ = std::chrono::high_resolution_clock::now();
  std::unordered_map<uint32_t, tf::Transform> object_poses = object_poses_;
  std::unordered_map<uint32_t, ros::Time> last_valid_times = last_valid_times_;
  std::unordered_map<uint32_t, tf::Transform> new_object_poses;
  for (unsigned int i = 0; i < detect_msg.detections.size(); i++) {
    auto object_pose = detect_msg.detections[i].results[0].pose.pose;
    // Add to ID for tracking multiple instances - Assumes a message is always one type of object
    int object_id = detect_msg.detections[i].results[0].id + (100 * i); 
    tf::Transform transform(
        tf::Quaternion(object_pose.orientation.x, object_pose.orientation.y,
                       object_pose.orientation.z, object_pose.orientation.w),
        tf::Vector3(object_pose.position.x, object_pose.position.y,
                    object_pose.position.z));    
    object_poses[object_id] = transform;
    new_object_poses[object_id] = transform;
    last_valid_times[object_id] = current_time;
  }
  object_poses_ = object_poses;
  last_valid_times_ = last_valid_times;
  return new_object_poses;
}

bool ObjectTracker::isConnected() { return detection_sub_.getNumPublishers() > 0; }

std::chrono::time_point<std::chrono::high_resolution_clock>
ObjectTracker::getTrackingTime() {
  return last_tracking_time_;
}
