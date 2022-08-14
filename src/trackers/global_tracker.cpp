#include "aerial_autonomy/trackers/global_tracker.h"
#include "aerial_autonomy/common/conversions.h"
#include <geometry_msgs/TransformStamped.h>
#include <glog/logging.h>

GlobalTracker::GlobalTracker(
            std::string tracker_type,
            parsernode::Parser &drone_hardware,
            tf::Transform camera_transform,
            tf::Transform tracking_offset_transform,
            double filter_gain_tracking_pose,
            double filter_gain_steps,
            bool fix_orientation,
            bool straight_line_orientation,
            double min_distance_between_objects,
            int min_detections, 
            SensorPtr<std::pair<tf::StampedTransform, tf::Vector3>> odom_sensor,
            std::chrono::duration<double> timeout,
            std::string name_space)
  : BaseTracker(std::move(std::unique_ptr<TrackingStrategy>(
            new ClosestTrackingStrategy(default_num_retries_)))),
    drone_hardware_(drone_hardware),
    camera_transform_(camera_transform),
    tracking_offset_transform_(tracking_offset_transform),
    odom_sensor_(odom_sensor),
    filter_gain_tracking_pose_(filter_gain_tracking_pose),
    filter_gain_steps_(filter_gain_steps),
    fix_orientation_(fix_orientation),
    straight_line_orientation_(straight_line_orientation),
    min_distance_between_objects_(min_distance_between_objects),
    min_detections_(min_detections),
    id_factor_(100),
    nh_(name_space)
{
  tracker_type_ = tracker_type; 
  if (tracker_type_ == "GlobalAlvar")
  {
    alvar_tracker_ = new AlvarTracker(timeout);
    tracker_sub_ = nh_.subscribe("ar_pose_marker", 1,
                          &GlobalTracker::alvarTrackerCallback, this);
  } 
  else if (tracker_type_ == "GlobalObject")
  {
    object_tracker_ = new ObjectTracker(timeout);
    tracker_sub_ = nh_.subscribe("object_detections", 1,
                          &GlobalTracker::objectTrackerCallback, this);
  } 
  else 
  {
    throw std::runtime_error("Unknown tracker type provided to GlobalTracker: " +
                              tracker_type_);
  }
}

tf::Transform GlobalTracker::filter(uint32_t id, tf::Transform input) {
  boost::mutex::scoped_lock lock(filter_mutex_);
  // If there isn't a filter for that key, add one
  if (tracking_pose_filters_.find(id) == tracking_pose_filters_.end())
  {
    tracking_pose_filters_.insert({id, DecayingExponentialFilter<PositionYaw>(filter_gain_tracking_pose_, filter_gain_steps_)});
  }

  PositionYaw tracking_position_yaw;
  conversions::tfToPositionYaw(tracking_position_yaw, input);
  PositionYaw filtered_position_yaw =
      tracking_pose_filters_.at(id).addAndFilter(tracking_position_yaw);
  tf::Transform filtered_pose;
  conversions::positionYawToTf(filtered_position_yaw, filtered_pose);
  return filtered_pose;
}

void GlobalTracker::resetFilters() {
  boost::mutex::scoped_lock lock(filter_mutex_);
  for (auto filter : tracking_pose_filters_)
  {
    (filter.second).reset();
  }
}

bool
GlobalTracker::vectorIsGlobal() {
  return true;
}

bool GlobalTracker::getTrackingVectors(
    std::unordered_map<uint32_t, tf::Transform> &pose) {
  
  if (!trackingIsValid()) {
    return false;
  }

  pose = target_poses_;

  std::unordered_map<uint32_t, int> num_detections = num_detections_;
  std::vector<uint32_t> objects_to_remove;
  // Remove poses without enough detections
  for (auto object : pose)
  {
    if (num_detections[object.first] < min_detections_)
    {
      objects_to_remove.emplace_back(object.first);
    }
  }
  for (auto key : objects_to_remove)
  {
    // Remove invalid objects
    pose.erase(key);
  }

  return true;
}

// Adapted to handle multiple objects with the same ID
bool GlobalTracker::getTrackingVector(std::tuple<uint32_t, tf::Transform> &pose) {

  bool valid = BaseTracker::getTrackingVector(pose);

  if (valid)
  {
    VLOG_EVERY_N(1, 100) << "TRACKING OBJECT: " << std::get<0>(pose);
    // Return just the base ID
    std::get<0>(pose) = std::get<0>(pose) % id_factor_;
  }
  
  return valid;
}

bool GlobalTracker::trackingIsValid() {
  if (tracker_type_ == "GlobalAlvar")
  {
    return alvar_tracker_->trackingIsValid();
  } 
  else if (tracker_type_ == "GlobalObject")
  {
    // Perform to object tracker poses - removes invalid poses
    object_tracker_->trackingIsValid();

    // Remove invalid poses and check validity of global tracker
    std::unordered_map<uint32_t, tf::Transform> target_poses = target_poses_;
    std::unordered_map<uint32_t, ros::Time> last_valid_times = last_valid_times_;
    std::vector<uint32_t> removed_objects;
    bool valid = object_tracker_->trackingIsValidAndRemoveInvalidPoses(target_poses, last_valid_times, removed_objects);

    // Additionally remove filters and num_detections
    boost::mutex::scoped_lock lock(filter_mutex_);
    std::unordered_map<uint32_t, int> num_detections = num_detections_;
    for (auto key : removed_objects)
    {
      // Remove invalid objects
      tracking_pose_filters_.erase(key);
      num_detections.erase(key);
    }

    target_poses_ = target_poses;
    last_valid_times_ = last_valid_times;
    num_detections_ = num_detections;
    
    return valid;
  } 
  return false;
}

std::chrono::time_point<std::chrono::high_resolution_clock>
GlobalTracker::getTrackingTime() {
  if (tracker_type_ == "GlobalAlvar")
  {
    return alvar_tracker_->getTrackingTime();
  } 
  else if (tracker_type_ == "GlobalObject")
  {
    return object_tracker_->getTrackingTime();
  } 
  return std::chrono::high_resolution_clock::now();
}

void GlobalTracker::objectTrackerCallback(
      const vision_msgs::Detection3DArray &tracker_msg)
{
  // Return if there are not any detections
  if (tracker_msg.detections.size() == 0)
    return;

  trackerCallback(tracker_msg.header, object_tracker_->getObjectPoses(tracker_msg));

  // std::unordered_map<uint32_t, tf::Transform> new_object_poses = object_tracker_->getObjectPoses(tracker_msg)
  // // Check if there are multiple instances of objects
  // // Assumes a message is always one type of object
  // if (tracker_msg.detections.size() > 1)
  // {
    
  // }


  // trackerCallback(tracker_msg.header, new_object_poses);
}

void GlobalTracker::alvarTrackerCallback(
      const ar_track_alvar_msgs::AlvarMarkers &tracker_msg)
{
  // Return if there are not any markers
  if (tracker_msg.markers.size() == 0)
    return;

  trackerCallback(tracker_msg.header, alvar_tracker_->getObjectPoses(tracker_msg));
}

void GlobalTracker::trackerCallback(const std_msgs::Header &header_msg, 
                      std::unordered_map<uint32_t, tf::Transform> new_object_poses)
{
  ros::Time current_time = ros::Time::now();

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

  // Recalculate tracking poses for any new detections
  std::unordered_map<uint32_t, tf::Transform> target_poses = target_poses_;
  std::unordered_map<uint32_t, tf::Transform> previous_target_poses = target_poses_;
  std::unordered_map<uint32_t, ros::Time> last_valid_times = last_valid_times_;
  std::unordered_map<uint32_t, int> num_detections = num_detections_;

  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp = header_msg.stamp;
  tf_msg.header.frame_id = "world";
  for (auto object : new_object_poses)
  {
    tf::Transform new_pose = 
    // target_poses[object.first] =
      quad_pose * camera_transform_ * object.second * tracking_offset_transform_;
    
    // Determine base id
    uint32_t base_id = object.first % id_factor_;

    // Calculate distance to current target poses of matching ID
    double min_dist = 1000;
    uint32_t matched_id = 1000;
    for (auto target : previous_target_poses)
    {
      if (target.first % id_factor_ == base_id)
      {
        double x_diff = new_pose.getOrigin().getX() - target.second.getOrigin().getX();
        double y_diff = new_pose.getOrigin().getY() - target.second.getOrigin().getY();
        double z_diff = new_pose.getOrigin().getZ() - target.second.getOrigin().getZ();
        double dist = sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
        if (dist < min_dist)
        {
          matched_id = target.first;
          min_dist = dist; 
        }
      }

    }

    // If not matched to an existing object or min distance greater than threshold, 
    // the create new object
    if ((matched_id == 1000) || (min_dist > min_distance_between_objects_))
    {
      // If there isn't an object with that key
      int multiplier = 0;
      matched_id = base_id;
      while (target_poses.find(matched_id) != target_poses.end())
      {
        multiplier += 1;
        matched_id = base_id + multiplier * id_factor_;
      }
      num_detections[matched_id] = 0; // Initialize
    }

    // Filter (and removes rp)
    target_poses[matched_id] = filter(matched_id, new_pose);

    // If estimate is relatively stable keep orientation fixed
    if (fix_orientation_ && tracking_pose_filters_.at(matched_id).isDecayComplete())
    {
      target_poses[matched_id].setRotation(previous_target_poses[matched_id].getRotation());
    }
    else if (straight_line_orientation_)
    {
      // Set orientation based on straight line to vehicle
      // Assume rotating around z axis
      double x_diff = target_poses[matched_id].getOrigin().getX() - quad_pose.getOrigin().getX();
      double y_diff = target_poses[matched_id].getOrigin().getY() - quad_pose.getOrigin().getY();
      double yaw = atan2(y_diff, x_diff);
      target_poses[matched_id].setRotation(tf::Quaternion(0, 0, sin(yaw/2), cos(yaw/2)));

      // Offset transform as desired
      target_poses[matched_id] = target_poses[matched_id] * tracking_offset_transform_;
    }

    last_valid_times[matched_id] = current_time;
    num_detections[matched_id] += 1;

    // Publish tf 
    tf_msg.child_frame_id = "object_" + std::to_string(matched_id);
    tf::transformTFToMsg(target_poses[matched_id], tf_msg.transform);
    br.sendTransform(tf_msg);
  }
  target_poses_ = target_poses;
  last_valid_times_ = last_valid_times;
  num_detections_ = num_detections;
}
