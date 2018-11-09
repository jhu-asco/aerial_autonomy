#pragma once
#include "aerial_autonomy/types/particle_state.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/reference_trajectory.h"
#include "aerial_autonomy/types/snap.h"
#include "base_controller_connector.h"
#include "obstacle_config.pb.h"
#include "qrotor_backstepping_controller_connector.h"

class ObstacleAvoidanceConnector
    : public ControllerConnector<std::pair<Position, ObstacleConfig>,
                                 PositionYaw,
                                 ReferenceTrajectoryPtr<ParticleState, Snap>> {
public:
  using ControllerType =
      Controller<std::pair<Position, ObstacleConfig>, PositionYaw,
                 ReferenceTrajectoryPtr<ParticleState, Snap>>;
  using BaseConnector =
      ControllerConnector<std::pair<Position, ObstacleConfig>, PositionYaw,
                          ReferenceTrajectoryPtr<ParticleState, Snap>>;
  ObstacleAvoidanceConnector(
      SensorPtr<std::pair<tf::StampedTransform, tf::Vector3>> odom_sensor,
      ControllerType &controller,
      QrotorBacksteppingControllerConnector &dependent_connector,
      ObstacleConfig obstacle_config);

protected:
  ObstacleConfig obstacle_config_;
  SensorPtr<std::pair<tf::StampedTransform, tf::Vector3>> odom_sensor_;
  QrotorBacksteppingControllerConnector &dependent_connector_;
  bool
  extractSensorData(std::pair<Position, ObstacleConfig> &sensor_data) override;
  void sendControllerCommands(
      ReferenceTrajectoryPtr<ParticleState, Snap> controls) override;
  void initialize() override;
  AbstractControllerConnector *getDependentConnector() override {
    return &dependent_connector_;
  }
};
