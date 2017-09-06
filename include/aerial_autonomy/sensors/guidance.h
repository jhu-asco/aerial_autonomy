#pragma once
#include "aerial_autonomy/sensors/base_sensor.h"
#include "parsernode/parser.h"

/**
* @brief sensor object which treats drone velocity
* data as external sensor data
*/
class Guidance : public Sensor<VelocityYaw> {
public:
  /**
  *
  * @brief Constructor
  *
  * @param drone_hardware UAV hardware
  *
  */
  Guidance(parsernode::Parser &drone_hardware)
      : Sensor(SensorStatus::Status::VALID), drone_hardware_(drone_hardware) {}

  virtual VelocityYaw getSensorData() {
    parsernode::common::quaddata data;
    drone_hardware_.getquaddata(data);

    VelocityYaw sensor_data;
    sensor_data.x = data.linvel.x;
    sensor_data.y = data.linvel.y;
    sensor_data.z = data.linvel.z;
    sensor_data.yaw = data.rpydata.y;

    return sensor_data;
  }

private:
  /**
  * @brief UAV hardware to get data from
  */
  parsernode::Parser &drone_hardware_;
};
