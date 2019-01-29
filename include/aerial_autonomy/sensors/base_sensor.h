#pragma once
#include "aerial_autonomy/types/sensor_status.h"
#include <aerial_autonomy/common/atomic.h>
#include <memory>

/**
* @brief Base class for sensors
*
* Subclass provides functionality to update sensor data
* and status
*/
template <class SensorDataT> class Sensor {
public:
  /**
  * @brief Constructor
  */
  Sensor() {}
  /**
  * @brief gets the latest raw sensor data
  */
  virtual SensorDataT getSensorData() = 0;
  /**
  * @brief gets the current status of the sensor
  */
  virtual SensorStatus getSensorStatus() = 0;
};

/**
* @brief Shared pointer to base sensor
*
* @tparam SensorDataT type of sensor data
*/
template <class SensorDataT>
using SensorPtr = std::shared_ptr<Sensor<SensorDataT>>;
