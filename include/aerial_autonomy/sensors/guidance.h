#pragma once
#include "aerial_autonomy/sensors/base_sensor.h"
#include "parsernode/parser.h"

/**
* @brief sensor object which treats drone velocity
* data as external sensor data
*/
class Guidance : public Sensor<std::tuple<VelocityYaw, Position>> {
public:
  /**
  *
  * @brief Constructor
  *
  * @param drone_hardware UAV hardware
  *
  */
  Guidance(parsernode::Parser &drone_hardware)
      : Sensor(SensorStatus::VALID), drone_hardware_(drone_hardware) {}

  virtual std::tuple<VelocityYaw, Position> getSensorData() {
    parsernode::common::quaddata data;
    drone_hardware_.getquaddata(data);

    VelocityYaw vel_data(data.linvel.x, data.linvel.y, data.linvel.z,
                         data.rpydata.z);

    Position pose_data(data.localpos.x, data.localpos.y, data.localpos.z);

    return std::make_tuple(vel_data, pose_data);
  }

private:
  /**
  * @brief UAV hardware to get data from
  */
  parsernode::Parser &drone_hardware_;
};
