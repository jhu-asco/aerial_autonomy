#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/controllers/velocity_based_position_controller.h"
#include "aerial_autonomy/controllers/rpyt_based_velocity_controller.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/roll_pitch_yaw_thrust.h"
#include "rpyt_based_position_controller_config.pb.h"

/**
 * @brief A position controller that sends rpyt commands to the hardware
 */
class RPYTBasedPositionController
    : public Controller<std::tuple<PositionYaw,VelocityYaw>, PositionYaw, RollPitchYawThrust> {
public:
  /**
  * @brief Constructor 
  */
  RPYTBasedPositionController(){}
    /**
  * @brief Constructor 
  */
  RPYTBasedPositionController(RPYTBasedPositionControllerConfig config): 
  vel_pos_ctlr(config.vel_pos_ctlr_config()), 
  rpyt_vel_ctlr(config.rpyt_vel_ctlr_config())
  {}
  /**
   * @brief Destructor
   */
  virtual ~RPYTBasedPositionController() {}

protected:
  /**
   * @brief Run the control loop.  Uses a rpyt controller to achieve the
   * desired position.
   * @param sensor_data Current position
   * @param goal Goal position
   * @param control RPYT command to send to hardware
   * @return true if RPYT command to reach goal is found
   */
  virtual bool runImplementation(std::tuple<PositionYaw, VelocityYaw> sensor_data, 
                                 PositionYaw goal,
                                 RollPitchYawThrust &control);
  /**
  * @brief Check if rpyt based position controller converged
  *
  * @param sensor_data Current position yaw
  * @param goal Goal position yaw
  *
  * @return  True if sensor data is close to goal
  */
  virtual ControllerStatus isConvergedImplementation(std::tuple<PositionYaw, VelocityYaw> sensor_data,
                                         PositionYaw goal);
  VelocityBasedPositionController vel_pos_ctlr; 
  RPYTBasedVelocityController rpyt_vel_ctlr;
};
