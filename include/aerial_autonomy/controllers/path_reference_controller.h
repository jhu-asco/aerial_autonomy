#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/path_sensor_trajectory.h"
#include "aerial_autonomy/types/reference_trajectory.h"
#include "particle_reference_config.pb.h"

#include <tf/tf.h>
#include <utility>
#include <chrono>

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

  virtual void setGoal(SensorPtr<PathReturnT> goal) {
    VLOG(1) << "Path Reference setGoal";
    Controller<std::pair<PositionYaw, tf::Transform>, SensorPtr<PathReturnT>,
               ReferenceTrajectoryPtr<Eigen::VectorXd,
                                      Eigen::VectorXd>>::setGoal(goal);
    goal_time_ = std::chrono::high_resolution_clock::now();
    VLOG(1) << "Path Reference goalTime is now: "  << std::chrono::duration<double>(goal_time_.time_since_epoch()).count();
  }
  
  virtual void reset() {
    VLOG(1) << "Path Reference Reset";
    //goal_time_ = std::chrono::high_resolution_clock::now();
  }

protected:
  /**
   * @brief generate reference trajectory based on goal
   *
   * @param goal sensor to use
   * @param control Reference in global frame
   *
   * @return true if able to generate the trajectory
   */
  bool runImplementation(
      std::pair<PositionYaw, tf::Transform> sensor_data,
      SensorPtr<PathReturnT> goal,
      ReferenceTrajectoryPtr<Eigen::VectorXd, Eigen::VectorXd> &control) {
    VLOG(1) << "Path Reference runImplementation";
    if (!control_) {
      VLOG(1) << "Path Reference Updating Pointer";
      control_.reset(new PathSensorTrajectory(goal, goal_time_));
    }
    control =
        ReferenceTrajectoryPtr<Eigen::VectorXd, Eigen::VectorXd>(control_);
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
  ReferenceTrajectoryPtr<Eigen::VectorXd, Eigen::VectorXd> control_;
  std::chrono::time_point<std::chrono::high_resolution_clock> goal_time_;
};
