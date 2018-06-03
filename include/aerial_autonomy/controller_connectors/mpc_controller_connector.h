#pragma once
#include "aerial_autonomy/controller_connectors/abstract_constraint_generator.h"
#include "aerial_autonomy/controller_connectors/base_controller_connector.h"
#include "aerial_autonomy/controllers/mpc_controller.h"

#include <Eigen/Dense>

#include <chrono>
#include <memory>

/**
* @brief Generic Controller connector for MPC controllers
*
* @tparam StateT The type of state used in MPC optimization
* @tparam ControlT The type of control sent to Robot
*/
template <class StateT, class ControlT>
class MPCControllerConnector
    : public ControllerConnector<MPCInputs<StateT>,
                                 ReferenceTrajectoryPtr<StateT, ControlT>,
                                 ControlT> {
  /**
  * @brief Parent controller connector
  */
  using BaseConnector =
      ControllerConnector<MPCInputs<StateT>,
                          ReferenceTrajectoryPtr<StateT, ControlT>, ControlT>;

public:
  /**
  * @brief Constructor.
  *
  * Initializes start time
  * @param controller The MPC Controller to use
  * @param constraint_generator The constraint generator to use
  * @param state_estimator state estimator that finds the state of robot
  * @param group The controller group. Used to ensure only one controller
  * runs per group.
  */
  MPCControllerConnector(
      AbstractMPCController<StateT, ControlT> &controller,
      ControllerGroup group,
      AbstractConstraintGeneratorPtr constraint_generator = nullptr)
      : BaseConnector(controller, group),
        constraint_generator_(constraint_generator),
        t_goal_(std::chrono::high_resolution_clock::now()) {}

  /**
  * @brief Estimate the current state and static params
  *
  * @param current_state Fill the state element
  * @param params Param vector to fill
  *
  * @return  True if estimation is successful
  */
  virtual bool estimateStateAndParameters(StateT &current_state,
                                          Eigen::VectorXd &params) = 0;

  /**
  * @brief extract the initial state and dynamic constraints
  *
  * @param mpc_inputs The inputs for MPC optimization
  *
  * @return true if state estimation and constraint generation is ok
  */
  virtual bool extractSensorData(MPCInputs<StateT> &mpc_inputs) {
    bool estimation_status = estimateStateAndParameters(
        mpc_inputs.initial_state, mpc_inputs.parameters);
    mpc_inputs.time_since_goal =
        std::chrono::duration<double>(
            std::chrono::high_resolution_clock::now() - t_goal_)
            .count();
    bool return_status = estimation_status;
    if (constraint_generator_) {
      mpc_inputs.constraints = constraint_generator_->generateConstraints();
      return_status = return_status && constraint_generator_->getStatus();
    }
    return return_status;
  }

  void setGoal(ReferenceTrajectoryPtr<StateT, ControlT> goal) {
    BaseConnector::setGoal(goal);
    t_goal_ = std::chrono::high_resolution_clock::now();
  }

  virtual void getTrajectory(std::vector<StateT> &xs,
                             std::vector<ControlT> &us) const = 0;

  virtual void getDesiredTrajectory(std::vector<StateT> &xds,
                                    std::vector<ControlT> &uds) const = 0;

private:
  AbstractConstraintGeneratorPtr
      constraint_generator_; ///< Generates constraints
  std::chrono::high_resolution_clock::time_point
      t_goal_; ///< Time when goal is set
};
