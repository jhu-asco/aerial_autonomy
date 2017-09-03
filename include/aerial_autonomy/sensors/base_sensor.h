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
enum class SensorStatus { INVALID = 0, VALID = 1 };
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
  virtual SensorDataT getSensorData() {
    SensorDataT sensor_data = sensor_data_;
    return sensor_data;
  };
  /**
  * @brief gets the current status of the sensor
  */
  SensorStatus getSensorStatus() {
    SensorStatus sensor_status = sensor_status_;
    return sensor_status;
  }

protected:
  /**
  * @brief Constructor that takes in sensor status
  */
  Sensor(SensorStatus sensor_status) : sensor_status_(sensor_status) {}
  /**
  * @brief variable to store sensor data
  */
  Atomic<SensorDataT> sensor_data_;
  /**
  * @brief variable to store the sensor status
  */
  Atomic<SensorStatus> sensor_status_;
};
