#pragma once
#include "aerial_autonomy/sensors/base_sensor.h"
#include "parsernode/parser.h"

/**
* @brief sensor object which treats drone velocity
* data as external sensor data
*/
class Guidance : public Sensor<Velocity> {
public:
  /**
  *
  * @brief Constructor
  *
  * @param drone_hardware UAV hardware
  *
  */
  Guidance(parsernode::Parser &drone_hardware)
      : drone_hardware_(drone_hardware) {}

  Velocity getSensorData() {
    parsernode::common::quaddata data;
    drone_hardware_.getquaddata(data);

    Velocity vel_data(data.linvel.x, data.linvel.y, data.linvel.z);
    return vel_data;
  }

  Velocity getTransformedSensorData() {
    // Assuming there is no sensor transform
    return getSensorData();
  }

  SensorStatus getSensorStatus() { return SensorStatus::VALID; }

private:
  /**
  * @brief UAV hardware to get data from
  */
  parsernode::Parser &drone_hardware_;
};
