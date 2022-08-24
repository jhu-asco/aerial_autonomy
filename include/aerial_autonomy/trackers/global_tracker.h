#pragma once
#include "aerial_autonomy/trackers/base_tracker.h"
#include "aerial_autonomy/trackers/closest_tracking_strategy.h"
#include "aerial_autonomy/sensors/base_sensor.h"
#include "aerial_autonomy/common/atomic.h"
#include "aerial_autonomy/filters/decaying_exponential_filter.h"
#include "aerial_autonomy/types/position_yaw.h"
#include <parsernode/parser.h>
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <boost/thread/mutex.hpp>

#include <vision_msgs/Detection3DArray.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <std_msgs/Header.h>

#include "aerial_autonomy/trackers/object_tracker.h"
#include "aerial_autonomy/trackers/alvar_tracker.h"

#include <chrono>
#include <ros/ros.h>

/**
 * @brief Global object tracker gives estimates in the global frame
 * Wraps an existing ROS tracker. Intended for Alvar and Object trackers
 */
class GlobalTracker : public BaseTracker {
public:
  /**
  * @brief Constructor that stores drone hardware
  * and camera transform for further use
  *
  * @param tracker_type Type of local tracker
  * @param drone_hardware UAV Hardware for getting sensor data
  * @param camera_transform Camera transform from UAV base
  * @param tracking_offset_transform Additional transform to apply to tracked
  * object before it is roll/pitch compensated
  * @param filter_gain_tracking_pose Gain for the pose filter
  * @param odom_sensor Odometry sensor to use if available
  * @param timeout Timeout before tracking is invalid
  * @param name_space Name space for node
  */
  GlobalTracker(std::string tracker_type,
                parsernode::Parser &drone_hardware,
                tf::Transform camera_transform,
                tf::Transform tracking_offset_transform = tf::Transform::getIdentity(),
                std::string tf_frame = "quad",
                double tf_time_offset = 0.0, 
                bool remove_time_since_last_measurement = false,
                double filter_gain_tracking_pose = 0.1,
                double filter_gain_steps = 10,
                bool fix_orientation = false,
                bool straight_line_orientation = false,
                double min_distance_between_objects = 1000,
                int min_detections = 1, 
                int id_factor = 100,
                SensorPtr<std::pair<tf::StampedTransform, tf::Vector3>> odom_sensor = nullptr,
                std::chrono::duration<double> timeout = std::chrono::milliseconds(500),
                std::string name_space = "~tracker");

  /**
   * @brief Get the tracking vectors
   * @param pos Returned tracking vectors
   * @return True if successful, false otherwise
   */
  virtual bool
  getTrackingVectors(std::unordered_map<uint32_t, tf::Transform> &pos);

  /**
   * @brief Get the tracking vector - overridden to handle multiple objects with the same ID
   * @param pose Returned tracking vector
   * @return True if successful, false otherwise
   */
  bool 
  getTrackingVector(std::tuple<uint32_t, tf::Transform> &pose);

  /**
  * @brief Transform tracking vectors from global poses to relative poses for tracking strategy
  * @param tracking_vectors Current tracking vectors 
  * @return Relative tracking vectors
  */
  std::unordered_map<uint32_t, tf::Transform> 
  relativeTrackingVectors(std::unordered_map<uint32_t, tf::Transform> tracking_vectors);

  /**
  * @brief Initialze the tracker.  
  * @return True if initialization succeeds, false otherwise
  */
  virtual bool initialize();

  virtual bool
  vectorIsGlobal();

  /**
  * @brief Check whether tracking is valid
  * @return True if the tracking is valid, false otherwise
  */
  virtual bool trackingIsValid();

  /**
  * @brief Get the time stamp of the current tracking vectors
  */
  virtual std::chrono::time_point<std::chrono::high_resolution_clock>
  getTrackingTime();

  /**
  * @brief Filter the position, yaw
  *
  * @param input pose
  *
  * @return filtered pose without roll, pitch
  */
  tf::Transform
  filter(uint32_t id, tf::Transform input);

  /**
  * @brief Reset all filters
  */
  virtual void
  resetFilters();

private:
  /**
  * @brief Tracker subscriber callback
  */
  void trackerCallback(const std_msgs::Header &header_msg, std::unordered_map<uint32_t, tf::Transform> new_object_poses);

  /**
  * @brief Object Tracker subscriber callback
  * @param tracker_msg Tracker array message
  */
  void objectTrackerCallback(const vision_msgs::Detection3DArray &tracker_msg);

  /**
  * @brief Alvar Tracker subscriber callback
  * @param tracker_msg Tracker array message
  */
  void alvarTrackerCallback(const ar_track_alvar_msgs::AlvarMarkers &tracker_msg);

  /**
  * @brief Get distance between poses
  * @param pose1 First pose
  * @param pose2 Second pose
  * @return distance between poses
  */
  double getDistance(tf::Transform pose1, tf::Transform pose2);

  parsernode::Parser &drone_hardware_; ///< UAV Hardware
  Atomic<std::unordered_map<uint32_t, tf::Transform>> target_poses_; ///< Tracked poses
  Atomic<std::unordered_map<uint32_t, ros::Time>> last_valid_times_; ///< Last time we received a pose
  Atomic<std::unordered_map<uint32_t, int>> num_detections_; ///< Number of detections of a pose
  tf::Transform camera_transform_; ///< Transform of camera in uav frame
  tf::Transform tracking_offset_transform_; ///< Transform to offset the tracking vector
  SensorPtr<std::pair<tf::StampedTransform, tf::Vector3>> odom_sensor_; ///< Odom sensor, if available
  std::unordered_map<uint32_t, DecayingExponentialFilter<PositionYaw>>
      tracking_pose_filters_; ///< Filter for tracking pose for each id
  mutable boost::mutex filter_mutex_; ///< Synchronize access to data
  double filter_gain_tracking_pose_; ///< Filter gain
  double filter_gain_steps_; ///< Filter gain steps to get to gain
  bool fix_orientation_; ///< Bool to fix the orientation after decay to avoid small changes
  bool straight_line_orientation_; ///< Bool to set orientation based on straight line from viewing pose
  double min_distance_between_objects_; ///< Any greater distance would be considered separate objects, set high to filter all poses together
  int min_detections_; ///< Minimum number of detections to consider a valid detection
  tf2_ros::TransformBroadcaster br; ///< TF Broadcaster
  tf2_ros::TransformListener *listener_;   ///< ros tf2 listener
  tf2_ros::Buffer buffer_; ///< ros tf2 buffer
  ros::Subscriber tracker_sub_;
  AlvarTracker *alvar_tracker_;
  ObjectTracker *object_tracker_;
  std::string tracker_type_;
  int id_factor_; ///< Factor to use for separate objects of the same ID
  ros::Duration time_since_last_message_;
  ros::Time last_message_time_;
  std::string tf_frame_;
  double tf_time_offset_;
  bool remove_time_since_last_measurement_;

  /**
  * @brief ROS node handle for communication
  */
  ros::NodeHandle nh_;

  /**
  * @brief Default number of retries for tracking a locked target
  */
  const int default_num_retries_ = 25;
};
