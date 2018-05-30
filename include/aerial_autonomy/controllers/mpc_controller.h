#pragma once
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/mpc_inputs.h"
#include "aerial_autonomy/types/reference_trajectory.h"

/**
* @brief Generic MPC controller.
*
* Specifies the sensordata, goal and control for the controller. Other
* MPC controllers should be a subclass of this Abstract MPC Controller.
*
*
* @tparam StateT The state type used in MPC optimization
* @tparam ControlT The control type used in MPC optimization
*/
template <class StateT, class ControlT>
class AbstractMPCController
    : public Controller<MPCInputs<StateT>,
                        ReferenceTrajectoryPtr<StateT, ControlT>, ControlT> {};
