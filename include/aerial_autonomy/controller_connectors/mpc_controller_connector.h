#pragma once
#include "aerial_autonomy/controller_connectors/abstract_constraint_generator.h"
#include "aerial_autonomy/controller_connectors/abstract_control_selector.h"
#include "aerial_autonomy/controller_connectors/base_controller_connector.h"
#include "aerial_autonomy/controllers/mpc_controller.h"
#include "aerial_autonomy/estimators/state_estimator.h"

/**
* @brief Generic Controller connector for MPC controllers
*
* @tparam StateT The type of state used in MPC optimization
* @tparam ControlT The type of control sent to Robot
*/
template <class StateT, class ControlT>
class MPCControllerConnector
    : public ControllerConnector<MPCInputs<StateT>,
                                 ReferenceTrajectory<StateT, ControlT>,
                                 ReferenceTrajectory<StateT, ControlT>> {
  /**
  * @brief Parent controller connector
  */
  using BaseConnector =
      ControllerConnector<MPCInputs<StateT>,
                          ReferenceTrajectory<StateT, ControlT>,
                          ReferenceTrajectory<StateT, ControlT>>;

public:
  /**
  * @brief Constructor.
  *
  * Initializes start time
  * \todo Gowtham create control selector based on proto strategy
  *
  * @param controller The MPC Controller to use
  * @param constraint_generator The constraint generator to use
  * @param state_estimator state estimator that finds the state of robot
  * @param group The controller group. Used to ensure only one controller
  * runs per group.
  */
  MPCControllerConnector(
      AbstractMPCController<StateT, ControlT> &controller,
      AbstractConstraintGenerator &constraint_generator,
      AbstractStateEstimator<StateT, ControlT> &state_estimator,
      ControllerGroup group)
      : BaseConnector(controller, group),
        constraint_generator_(constraint_generator),
        state_estimator_(state_estimator) {}

  /**
  * @brief extract the initial state and dynamic constraints
  *
  * @param mpc_inputs The inputs for MPC optimization
  *
  * @return true if state estimation and constraint generation is ok
  */
  virtual bool extractSensorData(MPCInputs<StateT> &mpc_inputs) {
    mpc_inputs.initial_state = state_estimator_.getState();
    mpc_inputs.constraints = constraint_generator_.generateConstraints();
    return (state_estimator_.getStatus() && constraint_generator_.getStatus());
  }

  /**
  * @brief send commands to hardware.
  *
  * @param control The control to send to hardware
  */
  virtual void sendCommandsToHardware(ControlT control) = 0;

  /**
  * @brief select controller from mpc optimization and update
  * estimator and send control to hardware
  *
  * @param trajectory The reference trajectory obtained from MPC optimization
  */
  void
  sendControllerCommands(ReferenceTrajectory<StateT, ControlT> trajectory) {
    StateT current_state_estimate = state_estimator_.getState();
    ControlT control =
        control_selector_->selectControl(trajectory, current_state_estimate);
    state_estimator_.propagate(control);
    sendCommandsToHardware(control);
  }

private:
  AbstractConstraintGenerator &constraint_generator_; ///< Generates constraints
  AbstractStateEstimator<StateT, ControlT>
      &state_estimator_; ///< Estimate current state
  std::unique_ptr<AbstractControlSelector<StateT, ControlT>>
      control_selector_; ///< Control selector
};
