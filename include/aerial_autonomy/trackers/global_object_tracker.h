#pragma once
#include "aerial_autonomy/trackers/object_tracker.h"
#include "aerial_autonomy/sensors/base_sensor.h"
#include "aerial_autonomy/common/atomic.h"
#include "aerial_autonomy/filters/decaying_exponential_filter.h"
#include "aerial_autonomy/types/position_yaw.h"
#include <parsernode/parser.h>
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <boost/thread/mutex.hpp>

/**
 * @brief Global object tracker gives estimates in the global frame
 */
class GlobalObjectTracker : public ObjectTracker {
public:
  /**
  * @brief Constructor that stores drone hardware
  * and camera transform for further use
  *
  * @param drone_hardware UAV Hardware for getting sensor data
  * @param camera_transform Camera transform from UAV base
  * @param tracking_offset_transform Additional transform to apply to tracked
  * object before it is roll/pitch compensated
  * @param filter_gain_tracking_pose Gain for the pose filter
  * @param odom_sensor Odometry sensor to use if available
  * @param timeout Timeout before tracking is invalid
  * @param name_space Name space for node
  */
  GlobalObjectTracker(parsernode::Parser &drone_hardware,
                tf::Transform camera_transform,
                tf::Transform tracking_offset_transform = tf::Transform::getIdentity(),
                double filter_gain_tracking_pose = 0.1,
                double filter_gain_steps = 10,
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

  virtual bool
  vectorIsGlobal();

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
  * @brief Detection subscriber callback
  * @param msg Detection array message
  */
  void detectionCallback(const vision_msgs::Detection3DArray &detect_msg);

  parsernode::Parser &drone_hardware_; ///< UAV Hardware
  Atomic<std::unordered_map<uint32_t, tf::Transform>> target_poses_; ///< Tracked poses
  tf::Transform camera_transform_; ///< Transform of camera in uav frame
  tf::Transform tracking_offset_transform_; ///< Transform to offset the tracking vector
  SensorPtr<std::pair<tf::StampedTransform, tf::Vector3>> odom_sensor_; ///< Odom sensor, if available
  std::unordered_map<uint32_t, DecayingExponentialFilter<PositionYaw>>
      tracking_pose_filters_; ///< Filter for tracking pose for each id
  mutable boost::mutex filter_mutex_; ///< Synchronize access to data
  double filter_gain_tracking_pose_; ///< Filter gain
  double filter_gain_steps_; ///< Filter gain steps to get to gain
  tf2_ros::TransformBroadcaster br; ///< TF Broadcaster
};
