#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/obstacle.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/quad_flat_output.h"
#include "aerial_autonomy/types/quad_flat_space_controls.h"
#include "aerial_autonomy/types/trajectory.h"
#include "aerial_autonomy/types/velocity_yaw_rate.h"
#include "mpc_higher_level_controller_config.pb.h"

#include <acado/function/function.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/user_interaction/user_interaction.hpp>
#include <acado/utils/acado_utils.hpp>
#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
/**
* @brief A higher level MPC controller that takes a
* goal state and obstacles positions and returns
* an obstacle free trajectory to the goal state
*/
class MPCHigherLevelController
    : public Controller<std::tuple<PositionYaw, std::vector<Obstacle>>,
                        Position,
                        std::tuple<Trajectory<QuadFlatOutput>,
                                   Trajectory<QuadFlatSpaceControls>>> {
public:
  /**
* @brief Constructor
*
* @param config Controller config
*
*/
  MPCHigherLevelController(MPCHigherLevelControllerConfig config);
  /**
  * @brief Implicit Constructor
  */
  MPCHigherLevelController()
      : MPCHigherLevelController(MPCHigherLevelControllerConfig()) {}
  /**
  * @brief return reference to ocp
  */
  const ACADO::OCP &getOCP() { return ocp_; }
  /**
  * @brief checks if the trajectory is feasible
  */
  bool checkTrajectoryFeasibility(
      std::tuple<PositionYaw, std::vector<Obstacle>> sensor_data,
      std::tuple<Trajectory<QuadFlatOutput>, Trajectory<QuadFlatSpaceControls>>
          &control);

  /**
  * @brief Deconstructor. Calls an ACADO-based function to clear the
  * static counters internally created by ACADO
  */
  virtual ~MPCHigherLevelController() { clearAllStaticCounters(); };

protected:
  /**
  * @brief Controller takes in initial position and obstacle centers
  * and the goal position and returns position-velocity trajectory
  *
  * @param sensor_data Tuple of initial position and list of obstacles
  * @param goal Goal position
  * @param control Trajectory of flat outputs and controls
  */
  virtual bool runImplementation(
      std::tuple<PositionYaw, std::vector<Obstacle>> sensor_data, Position goal,
      std::tuple<Trajectory<QuadFlatOutput>, Trajectory<QuadFlatSpaceControls>>
          &control);
  /**
  * @brief Check if trajectory ends at goal position
  *
  * @param sensor_data Tuple of initial position and
  * list of obstacles
  * @param goal Goal position of quad
  */
  virtual ControllerStatus isConvergedImplementation(
      std::tuple<PositionYaw, std::vector<Obstacle>> sensor_data,
      Position goal);

private:
  /**
  * Controller config
  */
  MPCHigherLevelControllerConfig config_;
  /**
  * States are quad position and upto 3rd derivative,
  * yaw and yaw rate
  */
  ACADO::DifferentialState x0, y0, z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, ga0,
      ga1;
  /**
  * Controls are snap and angular acceleration around world z-axis
  */
  ACADO::Control x4, y4, z4, ga2;
  /**
  * Dynamics
  */
  ACADO::Grid time_grid_; // time grid
  ACADO::DMatrix Q_;
  ACADO::OCP ocp_; // Optimal Control Problem
  std::unique_ptr<ACADO::OptimizationAlgorithm>
      algorithm_;                                     // Optimzation algorithm
  ACADO::VariablesGrid states_;                       // state trajectory
  ACADO::VariablesGrid controls_;                     // state controls
  std::tuple<Position, VelocityYawRate> final_state_; // end state of trajectory
};