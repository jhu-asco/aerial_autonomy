#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/position.h"
#include "position_controller_config.pb.h"

#include <tuple>

/**
 * @brief A position controller that keeps a position relative to some feedback
 * position
 */
class RelativePositionController
    : public Controller<std::tuple<Position, Position>, Position, Position> {
public:
  /**
  * @brief Constructor
  */
  RelativePositionController(PositionControllerConfig config)
      : config_(config) {}
  /**
   * @brief Destructor
   */
  virtual ~RelativePositionController() {}

protected:
  /**
   * @brief Run the control loop.  Uses a position controller to keep a desired
   * position relative to a tracked point.
   * @param sensor_data Position of controlled point and position of tracked
   * point
   * @param goal Goal relative position in control frame
   * @param control Position command
   * @return True if controller is successful in running
   */
  virtual bool runImplementation(std::tuple<Position, Position> sensor_data,
                                 Position goal, Position &control);
  /**
  * @brief Check if controller converged
  *
  * @param sensor_data Current control position and tracked position
  * @param goal Goal relative position
  *
  * @return True if converged
  */
  virtual bool
  isConvergedImplementation(std::tuple<Position, Position> sensor_data,
                            Position goal);

private:
  /**
  * @brief Config specifies position tolerance
  */
  PositionControllerConfig config_;
};
