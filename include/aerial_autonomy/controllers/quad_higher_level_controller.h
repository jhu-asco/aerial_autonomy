#pragma once
#include "acado_config.pb.h"
#include "aerial_autonomy/common/acado_higher_level_controller.h"
#include "aerial_autonomy/controllers/base_controller.h"
#include <glog/logging.h>

/**
* @brief Controller that generates a trajectory
* given initial & goal states and obstacle positions
*/
class QuadHigherLevelController
    : public Controller<std::tuple<QuadFlatOutput, std::vector<Obstacle>>,
                        QuadFlatOutput, Trajectory<QuadFlatOutput>> {
public:
  /**
  * @brief Constructor
  *
  * @param config Config for acado controller
  */
  QuadHigherLevelController(AcadoConfig config) : config_(config) {}
  /**
  * @brief Implicit constructor
  */
  QuadHigherLevelController() : QuadHigherLevelController(AcadoConfig()) {}
  /**
  * @brief Takes in a tuple of initial pose and list obstacle position,
  * and gives an obstacle-free trajectory to the goal pose
  *
  * @param sensor_data Tuple of intial and obstacle positions
  * @param goal Desired goal position
  * @param control Output trajectory
  */
  virtual bool runImplementation(
      std::tuple<QuadFlatOutput, std::vector<Obstacle>> sensor_data,
      QuadFlatOutput goal, Trajectory<QuadFlatOutput> &control);
  /**
  * @brief Checks if trajectory has converged to the desired goal
  *
  * @param sensor_data Dummy input. Uses final_state_ to check for convergence
  * @param goal Desired goal state.
  */
  virtual ControllerStatus isConvergedImplementation(
      std::tuple<QuadFlatOutput, std::vector<Obstacle>> sensor_data,
      QuadFlatOutput goal);

private:
  AcadoConfig config_;         ///< Config for acado controller
  QuadFlatOutput final_state_; ///< Variable to store final state
};
