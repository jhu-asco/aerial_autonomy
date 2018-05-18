#pragma once
#include "aerial_autonomy/controller_connectors/mpc_controller_connector.h"
#include "aerial_autonomy/types/quad_state.h"
#include "aerial_autonomy/types/roll_pitch_yaw_thrust.h"

#include <parsernode/parser.h>

/**
* @brief Controller connector for generic MPC Controller for Quadrotor system
*/
class MPCControllerDroneConnector
    : public MPCControllerConnector<QuadState, RollPitchYawThrust> {
public:
  /**
  * @brief Constructor
  *
  * @param drone_hardware Quadrotor parser that facilitates sending commands to
  * the robot
  * @param controller The MPC Controller to use
  * @param constraint_generator The constraint generator to use
  * @param state_estimator state estimator that finds the state of robot
  */
  MPCControllerDroneConnector(
      parsernode::Parser &drone_hardware,
      AbstractMPCController<QuadState, RollPitchYawThrust> &controller,
      AbstractConstraintGenerator &constraint_generator,
      AbstractStateEstimator<QuadState, RollPitchYawThrust> &state_estimator);
  /**
  * @brief send commands to Quadrotor
  *
  * @param control The commanded roll, pitch, yaw and thrust to send to
  * quadrotor
  */
  void sendCommandsToHardware(RollPitchYawThrust control);

private:
  parsernode::Parser
      &drone_hardware_; ///< Quadrotor parser for sending and receiving data
};
