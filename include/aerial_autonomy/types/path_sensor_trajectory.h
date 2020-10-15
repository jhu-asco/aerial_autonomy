#pragma once
#include <memory>
#include <utility>
#include "aerial_autonomy/types/reference_trajectory.h"
#include "aerial_autonomy/sensors/path_sensor.h"
#include <Eigen/Dense>

/**
* @brief A trajectory that wraps a path sensor.
*
*/
class PathSensorTrajectory
    : public ReferenceTrajectory<Eigen::VectorXd, Eigen::VectorXd> {

public:
  /**
   * @brief Constructor
   * 
   * @param path_sensor sensor that listens to ROS
   */

  PathSensorTrajectory(SensorPtr<PathReturnT> path_sensor);
  /**
  * @brief Gets the trajectory information at the specified time
  * @param t Time
  * @return Trajectory state and control
  */
  std::pair<Eigen::VectorXd, Eigen::VectorXd> atTime(double t) const;

  /**
   * @brief goal for reference trajectory
   *
   * @param t Time when goal is asked for
   *
   * @return state at time t
   */
  Eigen::VectorXd goal(double t) { return atTime(t).first; }

private:
  SensorPtr<PathReturnT> sensor_;
};
