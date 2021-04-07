#include "aerial_autonomy/sensors/bool_sensor.h"

bool BoolSensor::getSensorData() {
  return bool_;
}

void BoolSensor::boolCallback(const std_msgs::Bool::ConstPtr &bool_input) {
  bool_ = bool_input->data;
  last_time_ = ros::Time::now().toSec();
}

SensorStatus BoolSensor::getSensorStatus() {
  double curr_time = ros::Time::now().toSec();
  if ((curr_time - last_time_) > config_.timeout()) {
    return SensorStatus::INVALID;
  }
  return SensorStatus::VALID;
}
