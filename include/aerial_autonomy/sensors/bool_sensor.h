#pragma once
#include "aerial_autonomy/types/sensor_status.h"
#include <aerial_autonomy/common/atomic.h>
#include <aerial_autonomy/sensors/base_sensor.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>
#include <memory>
#include "ros_sensor_config.pb.h"
#include <Eigen/Dense>

/**
* @brief Bool sensor from a ros topic
* returns most recent bool value
*/

class BoolSensor : public Sensor<bool> {
public:
  /**
  * @brief Constructor
  *
  * @params config class
  */
BoolSensor(ROSSensorConfig config)
  : nh_(config.name_space()),bool_sub_(nh_.subscribe(config.topic(), 1, &BoolSensor::boolCallback, this)),
    config_(config){}


  /**
  * @brief Destructor
  */
  /**
  * @brief gets the latest sensor data
  *
  * @return most recent message
  */
  bool getSensorData();

  /**
  * @brief gets the current status of the sensor
  *
  * @return sensor status
  */
  SensorStatus getSensorStatus();

  /**
  * @brief ROS callback function
  *
  * @param path_input input ROS message
  */
  void boolCallback(const std_msgs::Bool::ConstPtr &bool_input);



private:
  ros::NodeHandle nh_;                             ///< Nodehandle
  ros::Subscriber bool_sub_;                       ///< ros subscriber
  Atomic<bool> bool_;              ///< latest bool
  Atomic<double> last_time_;              ///< latest time
  ROSSensorConfig config_;                        ///< Odom sensor config

};
