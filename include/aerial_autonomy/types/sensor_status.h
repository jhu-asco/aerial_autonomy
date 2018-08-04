#pragma once

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
bool sensor_status_to_bool(SensorStatus status);
