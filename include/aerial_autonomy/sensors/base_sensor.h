#pragma once
#include <aerial_autonomy/common/atomic.h>

/**
* @brief class for sensor status and related operations
*
* VALID : Sensor data is consistent with uav data upto
* certain degree of precision
*
* INVALID : Set if sensor data diverges from uav data
* for a prolonged period
*/
class SensorStatus {
public:
  /**
  * @brief Types of Statuses
  */
  enum class Status { INVALID, VALID };
  /**
  * @brief Default Constructor
  */
  SensorStatus() : status_(SensorStatus::Status::INVALID) {}
  /**
  * @brief Constructor that takes in a status
  */
  SensorStatus(Status status) : status_(status) {}
  /**
  * @brief Assignment operator from enum to SensorStatus
  * object
  */
  void operator=(const Status &status) { this->status_ = status; }
  /**
  * @brief Overloaded comparison operator to compare
  * class object with an enum
  *
  * @param sensor_status LHS is SensorStatus object
  * @param status RHS is Status object
  *
  * @returns true if sensor status has same enum as RHS
  */
  friend bool operator==(const SensorStatus &sensor_status,
                         const Status &status) {
    return sensor_status.status_ == status;
  }
  /**
* @brief Overloaded comparison operator to compare
* atomic class object with an enum
*
* @param sensor_status LHS is Atomic SensorStatus object
* @param status RHS is Status object
*
* @returns true if sensor status has same enum as RHS
*/
  friend bool operator==(const Atomic<SensorStatus> &sensor_status,
                         const Status &status) {
    SensorStatus temp_status = sensor_status;
    return temp_status.status_ == status;
  }
  /**
  * @brief overloaded comparison operator to compare
  * class object with another
  *
  * @param sensor_status LHS is first SensorStatus object
  * @param status RHS is second SensorStatus object
  *
  * @returns true if both have same status enum
  */
  friend bool operator==(const SensorStatus &lhs_sensor_status,
                         const SensorStatus &rhs_sensor_status) {
    return lhs_sensor_status.status_ == rhs_sensor_status.status_;
  }
  /**
  * @brief overloaded comparison operator to compare
  * class object with another
  *
  * @param sensor_status LHS is first SensorStatus object
  * @param status RHS is second SensorStatus object
  *
  * @returns true if both dont have same status enum
  */
  friend bool operator!=(const SensorStatus &lhs_sensor_status,
                         const SensorStatus &rhs_sensor_status) {
    return lhs_sensor_status.status_ != rhs_sensor_status.status_;
  }
  /**
  * @brief Explicit conversion of class to boolean variable
  *
  * @return true if sensor status is valid
  */
  explicit operator bool() const {
    return status_ == SensorStatus::Status::VALID;
  }

private:
  Status status_;
};
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
  Sensor() : sensor_status_(SensorStatus::Status::INVALID) {}
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
  Sensor(SensorStatus::Status status) : sensor_status_(status) {}
  /**
  * @brief variable to store sensor data
  */
  Atomic<SensorDataT> sensor_data_;
  /**
  * @brief variable to store the sensor status
  */
  Atomic<SensorStatus> sensor_status_;
};
