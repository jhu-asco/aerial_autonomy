#pragma once

struct CumulativeSensorStatus {
  enum class SensorStatus { Active = 0, Inactive = 1, Critical = 2 };
  SensorStatus external_pose_status = SensorStatus::Inactive;
  SensorStatus flight_status = SensorStatus::Inactive;
  SensorStatus guidance_status = SensorStatus::Inactive;
  SensorStatus tracker_status = SensorStatus::Inactive;
};

struct StateEstimatorStatus {
  enum class OptimizationStatus { SUCCESS = 0, FAILURE = 1 };
  CumulativeSensorStatus sensor_status;
  OptimizationStatus optimization_status = OptimizationStatus::SUCCESS;
  bool estimator_status = false;
};
