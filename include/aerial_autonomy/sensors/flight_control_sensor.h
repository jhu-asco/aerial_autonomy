#pragma once
#include "aerial_autonomy/sensors/base_sensor.h"
#include "parsernode/parser.h"

/**
* @brief sensor object which treats drone pose data as external sensor data
*/
class FlightControlSensor
    : public Sensor<std::pair<tf::StampedTransform, tf::Vector3>> {
public:
  /**
  *
  * @brief Constructor
  *
  * @param drone_hardware UAV hardware
  *
  */
  FlightControlSensor(parsernode::Parser &drone_hardware)
      : drone_hardware_(drone_hardware) {}

  std::pair<tf::StampedTransform, tf::Vector3> getSensorData() {
    parsernode::common::quaddata data;
    drone_hardware_.getquaddata(data);
    // Pose
    tf::StampedTransform quad_pose_stamped;
    const tf::Transform quad_pose(
        tf::createQuaternionFromRPY(data.rpydata.x, data.rpydata.y,
                                    data.rpydata.z),
        tf::Vector3(data.localpos.x, data.localpos.y, data.localpos.z));
    quad_pose_stamped.setData(quad_pose);
    // Linear velocity
    tf::Vector3 velocity(data.linvel.x, data.linvel.y, data.linvel.z);

    return std::make_pair(quad_pose_stamped, velocity);
  }

  SensorStatus getSensorStatus() { return SensorStatus::VALID; }

private:
  /**
  * @brief UAV hardware to get data from
  */
  parsernode::Parser &drone_hardware_;
};
