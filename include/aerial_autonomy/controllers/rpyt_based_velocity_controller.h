#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/velocity_yaw.h"
#include "aerial_autonomy/types/roll_pitch_yaw_thrust.h"
#include "rpyt_based_velocity_controller_config.pb.h"

/**
 * @brief A velocity controller that takes in rpyt commands 
 */

class RPYTBasedVelocityController 
: public Controller<VelocityYaw, VelocityYaw, RollPitchYawThrust>
{
public:
  /**
  * @brief Constructor with default config
  */
  RPYTBasedVelocityController():
  config_(RPYTBasedVelocityControllerConfig()){
  cumulative_err.x = 0.0;
  cumulative_err.y = 0.0;
  cumulative_err.z = 0.0;
  cumulative_err.yaw = 0.0;
  }

  /**
  * @brief Constructor which takes in a config
  */
  RPYTBasedVelocityController(RPYTBasedVelocityControllerConfig config):
  config_(config) {}

  virtual ~RPYTBasedVelocityController(){}

  protected:
  /**
   * @brief Run the control loop.  Uses a rpyt controller to achieve the
   * desired velocity.
   * @param sensor_data Current velocity
   * @param goal Goal velocity
   * @param control RPYT command to send to hardware
   * @return true if rpyt command to reach goal is found
   */
  virtual bool runImplementation(VelocityYaw sensor_data, VelocityYaw goal,
                                 RollPitchYawThrust &control);
  /**
  * @brief Check if RPYT based velocity controller converged
  *
  * @param sensor_data Current velocity yaw
  * @param goal Goal velocity yaw
  *
  * @return  True if sensor data is close to goal
  */
  virtual bool isConvergedImplementation(VelocityYaw sensor_data,
                                         VelocityYaw goal);
  RPYTBasedVelocityControllerConfig config_; ///< Controller configuration
  VelocityYaw cumulative_err;
  double dt = 0.03;  
};
