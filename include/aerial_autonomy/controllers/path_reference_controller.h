#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/path_sensor_trajectory.h"
#include "aerial_autonomy/types/reference_trajectory.h"
#include "particle_reference_config.pb.h"

#include <tf/tf.h>
#include <utility>

/**
 * @brief Controller that generates a reference trajectory based on
 *
 * a goal waypoint
 */
class PathReferenceController
    : public Controller<
          std::pair<PositionYaw, tf::Transform>, SensorPtr<PathReturnT>,
          ReferenceTrajectoryPtr<Eigen::VectorXd, Eigen::VectorXd>> {
public:
  /**
   * @brief constructor
   *
   * @param config particle reference config
   */
  PathReferenceController(){};

protected:
  /**
   * @brief generate reference trajectory based on goal
   *
   * @param goal sensor to use
   * @param control Reference in global frame
   *
   * @return true if able to generate the trajectory
   */
  /*virtual*/ bool runImplementation(
      std::pair<PositionYaw, tf::Transform> sensor_data, SensorPtr<PathReturnT> goal,
      ReferenceTrajectoryPtr<Eigen::VectorXd, Eigen::VectorXd> &control) {
          control.reset(new PathSensorTrajectory(goal));
          return true;
  }


  /**
   * @brief Status of the controller
   *
   * Always active
   *
   * @param std::pair   sensor data
   * @param PositionYaw goal
   *
   * @return  status of the controller
   */
  virtual ControllerStatus
      isConvergedImplementation(std::pair<PositionYaw, tf::Transform> sensor_data, 
                                SensorPtr<PathReturnT> goal){
    return ControllerStatus(ControllerStatus::Status::Active);
  }

private:
  //ReferenceTrajectoryPtr<Eigen::VectorXd, Eigen::VectorXd> control_;
};
