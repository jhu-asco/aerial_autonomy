#pragma once
#include "aerial_autonomy/controllers/mpc_controller.h"
#include "ddp_mpc_controller_config.pb.h"

// Gcop stuff
#include <gcop/casadi_system.h>
#include <gcop/ddp.h>
#include <gcop/loop_timer.h>
#include <gcop/lqcost.h>

#include <memory>

/**
* @brief DDP based MPC controller for generic casadi system
*
* Performs DDP on a Casadi model. Uses
* GCOP library for defining the system and performing optimization
*/
class DDPCasadiMPCController
    : public AbstractMPCController<Eigen::VectorXd, Eigen::VectorXd> {
public:
  /**
  * @brief Namespace for control type
  */
  using ControlType = Eigen::VectorXd;
  /**
  * @brief Namespace for state type
  */
  using StateType = Eigen::VectorXd;
  /**
  * @brief Namespace for goal type
  */
  using GoalType = ReferenceTrajectoryPtr<StateType, ControlType>;
  /**
  * @brief Constructor
  *
  * @param controller_duration The time step for calling controller
  */
  DDPCasadiMPCController(DDPMPCControllerConfig ddp_config,
                         std::chrono::duration<double> controller_duration);
  /**
  * @brief reset the controls to hovering and zero joint angles
  */
  void resetControls();

  /**
   * @brief Create a DDP using the system cost etc
   */
  void resetDDP();

  /**
   * @brief Set maximum iterations
   *
   * @param iters Max iterations.
   */
  void setMaxIters(int iters);

  /**
  * @brief Get maximum iterations
  *
  * @return Max iterations
  */
  int getMaxIters() const;

  /**
  * @brief Get MPC trajectory
  *
  * @param xs vector of states
  * @param us vector of controls
  */
  void getTrajectory(std::vector<StateType> &xs,
                     std::vector<ControlType> &us) const;

  /**
  * @brief Get reference MPC trajectory
  *
  * @param xds vector of states
  * @param uds vector of controls
  */
  void getDesiredTrajectory(std::vector<StateType> &xds,
                            std::vector<ControlType> &uds) const;

  /**
  * @brief Shift the controls such that control_new[0:N-shift_len] =
  * control_old[shift_len:N]
  * The remaining controls control_new[N-shift_len:] = control_old[N-1]
  *
  * @param shift_length The length to shift the controls by
  */
  void rotateControls(unsigned int shift_length);

  /**
  * @brief Get the average time taken to run MPC control loop
  *
  * @return the loop period in seconds
  */
  double getLoopTime() { return loop_timer_.average_loop_period(); }

  /**
  * @brief Get the MPC Cost from DDP
  *
  * @return Cost as double
  */
  double getMPCCost() { return ddp_->J; }

protected:
  /**
   * @brief Control that renders the system stationary
   *
   * @return the control
   */
  virtual ControlType stationaryControl() = 0;

  /**
   * @brief Find control to send to hardware after optimization
   *
   * @return  Control to send
   */
  virtual void outputControl(ControlType &control) = 0;

  /**
   * @brief Log data to datastream
   */
  virtual void logData(MPCInputs<StateType> &sensor_data,
                       ControlType &control) = 0;

protected:
  /**
  * @brief Run the MPC controller
  *
  * @param sensor_data MPC inputs i.e initial state, constraints, static
  * parameters
  * @param goal Goal reference trajectory
  * @param control Control to send to quadrotor
  *
  * @return
  */
  bool runImplementation(MPCInputs<StateType> sensor_data, GoalType goal,
                         ControlType &control);

protected:
  DDPMPCControllerConfig ddp_config_;                   ///< DDP Config
  std::unique_ptr<gcop::CasadiSystem<>> sys_;           ///< GCOP system
  std::unique_ptr<gcop::Ddp<Eigen::VectorXd>> ddp_;     ///< GCOP DDP optimizer
  std::unique_ptr<gcop::LqCost<Eigen::VectorXd>> cost_; ///< LQ cost function
  Eigen::VectorXd xf_;           ///< Not being used since xds and uds are used
  std::vector<StateType> xs_;    ///< vector of states
  std::vector<ControlType> us_;  ///< vector of controls
  std::vector<StateType> xds_;   ///< vector of reference states
  std::vector<ControlType> uds_; ///< vector of reference controls
  Eigen::VectorXd kt_;           ///< Thrust gain
  ControlType lb_;               ///< Lowerbound on control
  ControlType ub_;               ///< Lowerbound on control
  std::vector<double> ts_;       ///< vector of timestamps
  unsigned int
      look_ahead_index_shift_; ///< Future time stamp for controller being
                               /// passed out to account for controller delay
  unsigned int
      max_look_ahead_index_shift_; ///< Future time stamp for controller being
  /// passed out to account for controller delay
  LoopTimer
      loop_timer_; ///< Timer to find the average time taken by a controller
  unsigned int
      control_timer_shift_; ///< How many steps should the control shift by for
                            /// hot starting
  unsigned int max_iters_;  ///< Maximum number of iterations
  mutable boost::mutex
      copy_mutex_;                ///< Synchronize access to states and controls
  bool controller_config_status_; ///< If config provided is ok
};
