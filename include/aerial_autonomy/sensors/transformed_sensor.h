#pragma once
#include "aerial_autonomy/sensors/base_sensor.h"
#include "aerial_autonomy/types/sensor_status.h"
#include <aerial_autonomy/common/atomic.h>
#include <memory>
#include <ros/ros.h>

/**
* @brief Base class for sensors with local transformations
*
* Subclass provides functionality to update sensor data
* and status
*/
template <class SensorDataT>
class TransformedSensor : public Sensor<SensorDataT> {
public:
  /**
  * @brief Initialize the local transform
  */
  TransformedSensor(tf::Transform input) : local_transform_(input) {}
  /**
  * @brief gets the latest sensor data in the robot frame.
  */
  virtual SensorDataT getTransformedSensorData() = 0;

protected:
  /**
  * @brief The transform converting points in the sensor frame into the robot
  * frame.
  */
  tf::Transform local_transform_;
};
