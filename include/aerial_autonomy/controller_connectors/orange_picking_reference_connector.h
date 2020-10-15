#pragma once
#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/controller_connectors/base_controller_connector.h"
#include "aerial_autonomy/sensors/base_sensor.h"
#include "aerial_autonomy/sensors/path_sensor.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/reference_trajectory.h"
#include "aerial_autonomy/controllers/path_reference_controller.h"
#include "aerial_autonomy/controller_connectors/rpyt_based_reference_connector.h"
#include "aerial_autonomy/log/log.h"
#include "aerial_autonomy/types/sensor_status.h"

#include <tf/tf.h>
#include <utility> // Pair

#include <parsernode/parser.h>

/**
 * @brief An orange picking controller-connector 
 */
class OrangePickingReferenceConnector
    : public ControllerConnector<std::pair<PositionYaw, tf::Transform>,
                                 SensorPtr<PathReturnT>,
                                 ReferenceTrajectoryPtr<Eigen::VectorXd,Eigen::VectorXd>> {
public:

  OrangePickingReferenceConnector(parsernode::Parser &drone_hardware, 
        PathReferenceController &generator,
        RPYTBasedReferenceConnector<Eigen::VectorXd, Eigen::VectorXd> &dependent_connector,
        SensorPtr<std::pair<tf::StampedTransform, tf::Vector3>> odom_sensor = nullptr) :
    BaseClass(generator, ControllerGroup::HighLevel),
    dependent_connector_(dependent_connector),  odom_sensor_(odom_sensor),
    drone_hardware_(drone_hardware) {
      //logTrackerHeader("Orange_Picking_Reference_Connector");
    }

  void initialize() {
    BaseClass::initialize();
    VLOG(1) << "initializeing orange picking reference";
    if (odom_sensor_) {
      if (odom_sensor_->getSensorStatus() != SensorStatus::VALID) {
        LOG(WARNING) << "Pose Sensor Invalid! Cannot initialize orange picking connector";
        return;
      }
    }
    this->run();
  }

  virtual bool extractSensorData(std::pair<PositionYaw, tf::Transform> &sensor_data) {
    if (odom_sensor_) {
      if (odom_sensor_->getSensorStatus() != SensorStatus::VALID) {
        LOG(WARNING) << "Pose sensor invalid!";
        return false;
      }
    }
    /*if (!controller_.getGoal()->getSensorStatus() == SensorStatus::VALID) {
      return false;
    }*/
    //sensor_data is unnecessary.  It's a dummy value.
    return true;
  }

  virtual void sendControllerCommands(ReferenceTrajectoryPtr<Eigen::VectorXd, Eigen::VectorXd> control) {
    dependent_connector_.setGoal(control);
  }

  virtual ~OrangePickingReferenceConnector() {}

  AbstractControllerConnector *getDependentConnector() { return &dependent_connector_; }

private:
  /**
  * @brief Connector that runs RPYT control
  */
  RPYTBasedReferenceConnector<Eigen::VectorXd, Eigen::VectorXd> &dependent_connector_;
  /**
  * @brief Sensor for odometry
  */
  SensorPtr<std::pair<tf::StampedTransform, tf::Vector3>> odom_sensor_;
  /**
  * @brief Quad hardware to send commands
  */
  parsernode::Parser &drone_hardware_;
  
  using BaseClass =
      ControllerConnector<std::pair<PositionYaw, tf::Transform>, SensorPtr<PathReturnT>,
                 ReferenceTrajectoryPtr<Eigen::VectorXd, Eigen::VectorXd>>;

  

};
