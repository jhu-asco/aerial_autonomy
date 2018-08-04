#include "aerial_autonomy/types/sensor_status.h"

bool sensor_status_to_bool(SensorStatus status) {
  if (status == SensorStatus::INVALID)
    return false;
  else
    return true;
}
