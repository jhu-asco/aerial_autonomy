#include <acado/function/function.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/user_interaction/user_interaction.hpp>
#include <acado/utils/acado_utils.hpp>
#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>

#include "acado_config.pb.h"
#include "aerial_autonomy/types/obstacle.h"
#include "aerial_autonomy/types/quad_flat_output.h"
#include "aerial_autonomy/types/trajectory.h"
/**
* Class of Acado-Related functions for
*/
class AcadoHigherLevelController {
public:
  /**
  * @brief Implicit constructor
  */
  AcadoHigherLevelController() : AcadoHigherLevelController(AcadoConfig()) {}
  /**
  * @brief Constructor
  *
  * @param config Acado Config
  */
  AcadoHigherLevelController(AcadoConfig config);
  /**
  * @brief Solves the optimization problem
  *
  * @param sensor_data Tuple of initial state and obstacle positions
  * @param goal Goal position
  * @param control The output trajectory
  *
  * @returns True if optimization is succesful
  */

  bool
  solve(const std::tuple<QuadFlatOutput, std::vector<Obstacle>> &sensor_data,
        QuadFlatOutput goal, Trajectory<QuadFlatOutput> &control);
  /**
  * @brief Deconstructor Calls a ACADO-based function
  * to clear static counters interally created by ACADO
  */
  ~AcadoHigherLevelController() { clearAllStaticCounters(); }

private:
  /**
  * @brief Set initial state
  *
  * @param intial Intial State
  * @param goal Desired Final State
  */
  void setInitialandGoalState(QuadFlatOutput initial, QuadFlatOutput goal);
  AcadoConfig config_; ///< Acado Config
  ACADO::OCP ocp_;     ///< Optimal Control Problem
  ACADO::DifferentialState x0, y0, z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, ga0,
      ga1;                        ///< States
  ACADO::Control x4, y4, z4, ga2; ///< Controls
  std::unique_ptr<ACADO::OptimizationAlgorithm>
      algorithm_; ///< Optimization Algorithm
};