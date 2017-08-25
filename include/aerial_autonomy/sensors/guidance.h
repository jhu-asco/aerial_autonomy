#pragma once
#include "aerial_autonomy/sensors/base_sensor.h"
#include "parsernode/parser.h"

/**
* @brief sensor object which treats drone velocity 
* data as externa sensor data
*/
class Guidance : public Sensor<VelocityYaw>
{
public:
  /**
  * 
  * @brief Constructor
  *
  * @param UAV hardware
  *
  */
  Guidance(parsernode::Parser &drone_hardware) 
  : drone_hardware_(drone_hardware){}

  virtual void getSensorData(VelocityYaw sensor_data){
    parsernode::common::quaddata data;
    drone_hardware_.getquaddata(data);

    sensor_data.x = data.linvel.x;
    sensor_data.y = data.linvel.y;
    sensor_data.z = data.linvel.z;
    sensor_data.yaw = data.rpydata.y;

    sensor_data_ = sensor_data;
  }
};