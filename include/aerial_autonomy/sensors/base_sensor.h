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
  Sensor():sensor_status_(SensorStatus::INVALID){}
  /**
  * @brief gets the latest sensor data
  */
  SensorDataT getSensorData(){
    SensorDataT sensor_data = sensor_data_;
    return sensor_data;

  };
  /**
  * @brief gets the current status of the sensor
  */
  SensorStatus getSensorStatus(){
    SensorStatus sensor_status = sensor_status_;
    return sensor_status;
  }

  /**
  * @brief set sensor status
  */
  void setSensorStatus(SensorStatus sensor_status){
    sensor_status_ = sensor_status;
  }
protected:
  /**
  * @brief variable to store sensor data
  */
  Atomic<SensorDataT> sensor_data_;
  /**
  * @brief variable to store the sensor status
  */
  Atomic<SensorStatus> sensor_status_;
};
