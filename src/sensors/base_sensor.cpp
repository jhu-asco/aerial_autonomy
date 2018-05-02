#include "aerial_autonomy/sensors/base_sensor.h"

bool sensor_status_to_bool(SensorStatus status) {
  if (status == SensorStatus::INVALID)
    return false;
  else
    return true;
}

