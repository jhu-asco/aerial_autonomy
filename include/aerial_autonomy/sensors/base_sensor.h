#pragma once
#include <aerial_autonomy/common/atomic.h>

/**
* @brief enum for sensor status
*/
enum SensorStatus
{
  VALID = 0,
  INVALID = 1
};
/**
* @brief Base class for sensors
*
* Subclass provides functionality to update sensor data 
* and status 
*/
template<class SensorDataT>
class Sensor{
public:
  /**
  * @brief Constructor 
  */
  Sensor(){}
  /**
  * @brief gets the latest sensor data
  */
  void getSensorData(SensorDataT &sensor_data){
    sensor_data = sensor_data_;
  };
  /**
  * @brief gets the current status of the sensor
  */
  void getSensorStatus(SensorStatus &sensor_status){
    sensor_status =  sensor_status_;
  }
protected:
  /**
  * @brief variable to store sensor data
  */
  Atomic<SensorDataT> sensor_data_;
  /**
  * @brief variable to store the sensor status
  */
  SensorStatus sensor_status_;
};
