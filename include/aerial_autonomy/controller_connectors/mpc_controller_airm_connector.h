#pragma once
#include "aerial_autonomy/controller_connectors/mpc_controller_connector.h"
#include "aerial_autonomy/types/arm_state.h"
#include "aerial_autonomy/types/quad_state.h"
#include "aerial_autonomy/types/roll_pitch_yaw_thrust.h"
#include <arm_parsers/arm_parser.h>

#include <parsernode/parser.h>

/**
* @brief Controller connector for generic MPC Controller for Quadrotor and arm
* system
*/
class MPCControllerAirmConnector
    : public MPCControllerConnector<
          std::pair<QuadState, ArmState>,
          std::pair<RollPitchYawThrust, std::vector<double>>> {
public:
  using ControlType = std::pair<RollPitchYawThrust, std::vector<double>>;
  using StateType = std::pair<QuadState, ArmState>;
  /**
  * @brief Constructor
  *
  * @param drone_hardware Quadrotor parser that facilitates sending commands to
  * the robot
  * @param controller The MPC Controller to use
  * @param constraint_generator The constraint generator to use
  * @param state_estimator state estimator that finds the state of robot
  */
  MPCControllerAirmConnector(
      parsernode::Parser &drone_hardware, ArmParser &arm_hardware,
      AbstractMPCController<StateType, ControlType> &controller,
      AbstractConstraintGenerator &constraint_generator,
      AbstractStateEstimator<StateType, ControlType> &state_estimator);
  /**
  * @brief send commands to Quadrotor and arm
  *
  * @param control The commanded roll, pitch, yaw and thrust to send to
  * quadrotor and joint angles to send to arm
  */
  void sendCommandsToHardware(ControlType control);

private:
  parsernode::Parser
      &drone_hardware_;     ///< Quadrotor parser for sending and receiving data
  ArmParser &arm_hardware_; ///< Parser for sending and receiving arm data
};
