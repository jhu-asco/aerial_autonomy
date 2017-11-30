#pragma once
#include <aerial_autonomy/common/atomic.h>

/**
* @brief enum for sensor status
*
* VALID : Sensor data is consistent with uav data upto
* certain degree of precision
*
* INVALID : Set if sensor data diverges from uav data
* for a prolonged period
*/
enum class SensorStatus { INVALID, VALID };
/**
* @brief convert sensor status to bool
*/
bool sensor_status_to_bool(SensorStatus status) {
  if (status == SensorStatus::INVALID)
    return false;
  else
    return true;
}
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
  * @brief gets the latest sensor data
  */
  virtual SensorDataT getSensorData() = 0;
  /**
  * @brief gets the current status of the sensor
  */
  virtual SensorStatus getSensorStatus() = 0;
};
