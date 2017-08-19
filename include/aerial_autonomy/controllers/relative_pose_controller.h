#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/position.h"
#include "pose_controller_config.pb.h"

#include <tuple>

#include <tf/tf.h>

/**
 * @brief A pose controller that keeps a pose relative to some feedback
 * pose
 */
class RelativePoseController
    : public Controller<std::tuple<tf::Transform, tf::Transform>, tf::Transform,
                        tf::Transform> {
public:
  /**
  * @brief Constructor
  */
  RelativePoseController(PoseControllerConfig config) : config_(config) {}
  /**
   * @brief Destructor
   */
  virtual ~RelativePoseController() {}

protected:
  /**
   * @brief Run the control loop.  Uses a pose controller to keep a desired
   * pose relative to a tracked pose.
   * @param sensor_data Pose of controlled point and pose of tracked
   * pose
   * @param goal Goal relative pose in tracked pose frame
   * @param control Pose command
   * @return True if controller is successful in running
   */
  virtual bool
  runImplementation(std::tuple<tf::Transform, tf::Transform> sensor_data,
                    tf::Transform goal, tf::Transform &control);
  /**
  * @brief Check if controller converged
  *
  * @param sensor_data Current control pose and tracked pose
  * @param goal Goal relative pose in tracked pose frame
  *
  * @return status that contains different states the controller and debug info.
  */
  virtual ControllerStatus isConvergedImplementation(
      std::tuple<tf::Transform, tf::Transform> sensor_data, tf::Transform goal);

private:
  /**
  * @brief Config specifies position tolerance
  */
  PoseControllerConfig config_;
};
