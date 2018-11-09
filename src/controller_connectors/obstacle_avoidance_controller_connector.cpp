#include <aerial_autonomy/controller_connectors/obstacle_avoidance_controller_connector.h>

ObstacleAvoidanceConnector::ObstacleAvoidanceConnector(
    SensorPtr<std::pair<tf::StampedTransform, tf::Vector3>> odom_sensor,
    ControllerType &controller,
    QrotorBacksteppingControllerConnector &dependent_connector,
    ObstacleConfig obstacle_config)
    : BaseConnector(controller, ControllerGroup::HighLevel),
      obstacle_config_(obstacle_config), odom_sensor_(odom_sensor),
      dependent_connector_(dependent_connector) {}

bool ObstacleAvoidanceConnector::extractSensorData(
    std::pair<Position, ObstacleConfig> &sensor_data) {
  if (odom_sensor_->getSensorStatus() != SensorStatus::VALID) {
    return false;
  }
  auto data = odom_sensor_->getSensorData();
  tf::StampedTransform pose = data.first;
  Position quad_position(pose.getOrigin().x(), pose.getOrigin().y(),
                         pose.getOrigin().z());
  sensor_data = std::make_pair(quad_position, obstacle_config_);
  return true;
}

void ObstacleAvoidanceConnector::sendControllerCommands(
    ReferenceTrajectoryPtr<ParticleState, Snap> controls) {
  dependent_connector_.setGoal(controls);
}

void ObstacleAvoidanceConnector::initialize() { run(); }
