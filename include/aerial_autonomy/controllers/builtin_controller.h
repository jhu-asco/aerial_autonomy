#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/velocity_yaw.h"
#include "position_controller_config.pb.h"
#include "velocity_controller_config.pb.h"

/**
 * @brief A controller that simply outputs the set goal
 */
template <class GoalType>
class BuiltInController : public Controller<GoalType, GoalType, GoalType> {
public:
  /**
   * @brief Destructor
   */
  virtual ~BuiltInController() {}

protected:
  /**
   * @brief Run the control loop.  Simply returns the goal.
   * @param sensor_data Empty sensor data struct since no sensing is required.
   * @param goal Goal set-point
   * @param control Goal to send to hardware
   * @return Always true
   */
  virtual bool runImplementation(GoalType, GoalType goal, GoalType &control) {
    control = goal;
    return true;
  }
};

/**
* @brief Builtin position controller
*/
class BuiltInPositionController : public BuiltInController<PositionYaw> {
public:
  /**
  * @brief Constructor that store position controller configuration
  *
  * @param config specifies position and yaw tolerance
  */
  BuiltInPositionController(PositionControllerConfig config)
      : config_(config) {}

  /**
  * @brief Constructor that uses default constructor
  */
  BuiltInPositionController()
      : BuiltInPositionController(PositionControllerConfig()) {}

protected:
  /**
  * @brief Check if current position yaw is close to Goal position yaw
  * Uses position controller config
  *
  * @param current_position_yaw Current position and yaw from UAV
  * @param goal Goal position and yaw
  *
  * @return True if converged/False otherwise
  */
  virtual bool isConvergedImplementation(PositionYaw current_position_yaw,
                                         PositionYaw goal) {
    PositionYaw position_yaw_diff = current_position_yaw - goal;
    const double &tolerance_pos = config_.goal_position_tolerance();
    const double &tolerance_yaw = config_.goal_yaw_tolerance();
    // Compare
    if (std::abs(position_yaw_diff.x) < tolerance_pos &&
        std::abs(position_yaw_diff.y) < tolerance_pos &&
        std::abs(position_yaw_diff.z) < tolerance_pos &&
        std::abs(position_yaw_diff.yaw) < tolerance_yaw) {
      VLOG(1) << "Reached goal";
      return true;
    }
    return false;
  }

private:
  /**
  * @brief Config specifies position and yaw tolerance
  */
  PositionControllerConfig config_;
};

/**
* @brief Builtin velocity controller
*/
class BuiltInVelocityController : public BuiltInController<VelocityYaw> {
public:
  /**
  * @brief Constructor to store velocity controller configuration
  *
  * @param config Configuration about velocity and yaw tolerance
  */
  BuiltInVelocityController(VelocityControllerConfig config)
      : config_(config) {}

  /**
  * @brief Constructor with default configuration
  */
  BuiltInVelocityController()
      : BuiltInVelocityController(VelocityControllerConfig()) {}

protected:
  /**
  * @brief Check if current velocity yaw is close to Goal velocity yaw
  * Uses velocity controller config
  *
  * @param current_velocity_yaw Current velocity and yaw from UAV
  * @param goal Goal velocity and yaw
  *
  * @return True if converged/False otherwise
  */
  virtual bool isConvergedImplementation(VelocityYaw current_velocity_yaw,
                                         VelocityYaw goal) {
    VelocityYaw velocity_yaw_diff = current_velocity_yaw - goal;
    const double &tolerance_pos = config_.goal_velocity_tolerance();
    const double &tolerance_yaw = config_.goal_yaw_tolerance();
    // Compare
    if (std::abs(velocity_yaw_diff.x) < tolerance_pos &&
        std::abs(velocity_yaw_diff.y) < tolerance_pos &&
        std::abs(velocity_yaw_diff.z) < tolerance_pos &&
        std::abs(velocity_yaw_diff.yaw) < tolerance_yaw) {
      VLOG(1) << "Reached goal";
      return true;
    }
    return false;
  }

private:
  /**
  * @brief Configuration specifies velocity tolerance and yaw tolerance
  */
  VelocityControllerConfig config_;
};
