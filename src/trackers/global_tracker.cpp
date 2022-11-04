#include "aerial_autonomy/trackers/global_tracker.h"
#include "aerial_autonomy/common/conversions.h"
#include <geometry_msgs/TransformStamped.h>
#include <glog/logging.h>

GlobalTracker::GlobalTracker(
            std::string tracker_type,
            parsernode::Parser &drone_hardware,
            tf::Transform camera_transform,
            tf::Transform tracking_offset_transform,
            std::string tf_frame,
            double tf_time_offset,
            bool remove_time_since_last_measurement,
            double filter_gain_tracking_pose,
            double filter_gain_steps,
            bool fix_orientation,
            bool straight_line_orientation,
            double min_distance_between_objects,
            int min_detections, 
            int id_factor,
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
    id_factor_(id_factor),
    nh_(name_space)
{
  tf_frame_ = tf_frame;
  tf_time_offset_ = tf_time_offset;
  last_message_time_ = ros::Time::now();
  time_since_last_message_ = ros::Duration(0);
  remove_time_since_last_measurement_ = remove_time_since_last_measurement;

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

  listener_ = new tf2_ros::TransformListener(buffer_);
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

  // bool valid = BaseTracker::getTrackingVector(pose);

  if (!trackingIsValid()) {
    return false;
  }

  std::unordered_map<uint32_t, tf::Transform> tracking_vectors;
  if (!getTrackingVectors(tracking_vectors)) {
    return false;
  }

  // Transform tracking vectors for tracking strategies that assume relative poses
  // (such as ClosestTrackingStrategy)
  std::unordered_map<uint32_t, tf::Transform> 
    relative_tracking_vectors = relativeTrackingVectors(tracking_vectors);

  std::tuple<uint32_t, tf::Transform> relative_pose;
  if (!tracking_strategy_->getTrackingVector(relative_tracking_vectors, relative_pose)) {
    return false;
  }

  // Restore pose to global transform and return just the base ID
  uint32_t pose_id = std::get<0>(relative_pose);
  uint32_t base_pose_id = pose_id % id_factor_;
  pose = std::make_tuple(base_pose_id, tracking_vectors[pose_id]);

  VLOG_EVERY_N(1, 100) << "TRACKING OBJECT: " << pose_id;
  
  return true;
}

bool GlobalTracker::initialize() {
  std::unordered_map<uint32_t, tf::Transform> tracking_vectors;
  if (!getTrackingVectors(tracking_vectors)) {
    return false;
  }

  // Transform tracking vectors for tracking strategies that assume relative poses
  // (such as ClosestTrackingStrategy)
  std::unordered_map<uint32_t, tf::Transform> 
    relative_tracking_vectors = relativeTrackingVectors(tracking_vectors);

  return tracking_strategy_->initialize(relative_tracking_vectors);
}

std::unordered_map<uint32_t, tf::Transform> 
GlobalTracker::relativeTrackingVectors(std::unordered_map<uint32_t, tf::Transform> tracking_vectors)
{
  std::unordered_map<uint32_t, tf::Transform> relative_tracking_vectors = tracking_vectors;

  // Get current vehicle position
  parsernode::common::quaddata quad_data;
  drone_hardware_.getquaddata(quad_data);
  tf::Transform quad_pose;
  if (odom_sensor_) {
    if (odom_sensor_->getSensorStatus() != SensorStatus::VALID) {
      LOG(WARNING) << "Pose sensor invalid!";
      return relative_tracking_vectors;
    }
    quad_pose = odom_sensor_->getSensorData().first;
  } else {
    quad_pose = conversions::getPose(quad_data);
  }

  // Make poses relative 
  for (auto pose : tracking_vectors)
  {
    relative_tracking_vectors[pose.first] = quad_pose.inverse() * pose.second; 
  }

  return relative_tracking_vectors;
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

void GlobalTracker::resetTrackingVectors()
{
  if (tracker_type_ == "GlobalObject")
  {
    std::chrono::duration<double> original_timeout = object_tracker_->getTimeout();
    object_tracker_->setTimeout(std::chrono::milliseconds(0));
    
    // Call is valid for it to clear invalid poses
    trackingIsValid();

    // Set timeout back
    object_tracker_->setTimeout(original_timeout);
  }
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
  time_since_last_message_ = tracker_msg.header.stamp - last_message_time_;
  last_message_time_ = tracker_msg.header.stamp;

  // Return if there are not any detections
  if (tracker_msg.detections.size() == 0)
    return;

  trackerCallback(tracker_msg.header, object_tracker_->getObjectPoses(tracker_msg));
}

void GlobalTracker::alvarTrackerCallback(
      const ar_track_alvar_msgs::AlvarMarkers &tracker_msg)
{
  time_since_last_message_ = tracker_msg.header.stamp - last_message_time_;
  last_message_time_ = tracker_msg.header.stamp;

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
  // Find pose of vehicle with some offset of time for calculation time 
  geometry_msgs::TransformStamped pose_input;
  ros::Time tf_time = header_msg.stamp - ros::Duration(tf_time_offset_);
  if (remove_time_since_last_measurement_)
  {
    if (time_since_last_message_ < ros::Duration(1))
    {
      tf_time -= time_since_last_message_;
    }
    else
    {
      LOG(WARNING) << "Time since last tracker message: " << time_since_last_message_;
    }
  }
  try{
    pose_input = buffer_.lookupTransform("world", tf_frame_,
                              tf_time);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    // ros::Duration(0.1).sleep();
    return;
  }
  tf::StampedTransform quad_pose_stamped;
  tf::transformStampedMsgToTF(pose_input, quad_pose_stamped);
  tf::Transform quad_pose = quad_pose_stamped;

  // parsernode::common::quaddata quad_data;
  // drone_hardware_.getquaddata(quad_data);
  // tf::Transform quad_pose;
  // if (odom_sensor_) {
  //   if (odom_sensor_->getSensorStatus() != SensorStatus::VALID) {
  //     LOG(WARNING) << "Pose sensor invalid!";
  //     return;
  //   }
  //   quad_pose = odom_sensor_->getSensorData().first;
  // } else {
  //   quad_pose = conversions::getPose(quad_data);
  // }

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
    uint32_t matched_id_init = 1000;
    uint32_t matched_id = matched_id_init;
    for (auto target : previous_target_poses)
    {
      if (target.first % id_factor_ == base_id)
      {
        double dist = getDistance(new_pose, target.second);
        // If new closest object or closer than minimum distance between objects
        if ((dist < min_dist) || (dist < min_distance_between_objects_))
        {
          // Compare current estimate with previous best
          if (matched_id != matched_id_init)
          {
            double dist_btw_targets = getDistance(target.second, previous_target_poses[matched_id]);
            // If these two targets are close together
            if (dist_btw_targets < min_distance_between_objects_) 
            {
              // If the previous estimate has more detections, continue
              if (num_detections[matched_id] > num_detections[target.first])
              {
                LOG(WARNING) << "Adding to object with more detections";
                continue;
              }
            }
            // Otherwise, distance is greater, so only store if new smallest distance
            else if (dist >= min_dist)
            {
              continue;
            }
          }
          matched_id = target.first;
          min_dist = dist; 
        }
      }
    }

    // If not matched to an existing object or min distance greater than threshold, 
    // the create new object
    if ((matched_id == matched_id_init) || (min_dist > min_distance_between_objects_))
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

double GlobalTracker::getDistance(tf::Transform pose1, tf::Transform pose2)
{
  double x_diff = pose1.getOrigin().getX() - pose2.getOrigin().getX();
  double y_diff = pose1.getOrigin().getY() - pose2.getOrigin().getY();
  double z_diff = pose1.getOrigin().getZ() - pose2.getOrigin().getZ();
  return sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
}
