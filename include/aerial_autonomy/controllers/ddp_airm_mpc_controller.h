#pragma once
#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/controllers/mpc_controller.h"
#include "aerial_autonomy/types/arm_state.h"
#include "aerial_autonomy/types/quad_state.h"
#include "aerial_autonomy/types/roll_pitch_yaw_thrust.h"
#include "airm_mpc_controller_config.pb.h"

// Gcop stuff
#include <gcop/aerial_manipulation_feedforward_system.h>
#include <gcop/airm_residual_network_model.h>
#include <gcop/casadi_system.h>
#include <gcop/ddp.h>
#include <gcop/loop_timer.h>
#include <gcop/lqcost.h>

#include <glog/logging.h>

#include <memory>

/**
* @brief DDP based MPC controller for Aerial manipulation system
*
* Performs DDP on a Quadrotor model with a 2DOF arm attached. Uses
* GCOP library for defining the system and performing optimization
*/
class DDPAirmMPCController
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
  * @param config MPC controller config
  * @param controller_duration The time step for calling controller
  */
  DDPAirmMPCController(AirmMPCControllerConfig config,
                       std::chrono::duration<double> controller_duration);
  /**
  * @brief reset the controls to hovering and zero joint angles
  */
  void resetControls();

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
  * @brief Load quadrotor model parameters from text file
  *
  * @param kp_rpy Second order kp gain on rotation control
  * @param kd_rpy Second order kd gain on rotation control
  * @param p The thrust gain for quad model
  * @param folder_path The folder where text file is present
  */
  void loadQuadParameters(Eigen::Vector3d &kp_rpy, Eigen::Vector3d &kd_rpy,
                          Eigen::VectorXd &p, std::string folder_path);

  /**
  * @brief Loard arm model parameters from text file
  *
  * @param kp_ja Second order kp gain on joint angles
  * @param kd_ja Second order kd gain on joint angles
  * @param folder_path The folder where text file is present
  */
  void loadArmParameters(Eigen::Vector2d &kp_ja, Eigen::Vector2d &kd_ja,
                         std::string folder_path);

  /**
  * @brief shif the controls such that control_new[0:N-shift_len] =
  * control_old[shift_len:N]
  * The remaining controls control_new[N-shift_len:] = control_old[N-1]
  *
  * @param shift_length The length to shift the controls by
  */
  void rotateControls(int shift_length);

  /**
  * @brief Get the average time taken to run MPC control loop
  *
  * @return the loop period in seconds
  */
  double getLoopTime() { return loop_timer_.average_loop_period(); }

  /**
  * @brief Overwrite the MPC Config
  *
  * @param config MPC controller config
  */
  void setConfig(AirmMPCControllerConfig config);

  /**
  * @brief Get the MPC Cost from DDP
  *
  * @return Cost as double
  */
  double getMPCCost() { return ddp_->J; }

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
  /**
  * @brief Check if MPC converged
  *
  * @param sensor_data MPC inputs i.e initial state, constraints, static
  * parameters
  * @param goal Goal reference trajectory
  *
  * @return Controller status
  */
  ControllerStatus isConvergedImplementation(MPCInputs<StateType> sensor_data,
                                             GoalType goal);

private:
  AirmMPCControllerConfig config_;                  ///< MPC controller config
  std::unique_ptr<gcop::CasadiSystem<>> sys_;       ///< GCOP system
  std::unique_ptr<gcop::Ddp<Eigen::VectorXd>> ddp_; ///< GCOP DDP optimizer
  std::unique_ptr<gcop::LqCost<Eigen::VectorXd>> cost_; ///< LQ cost function
  Eigen::VectorXd xf_;           ///< Not being used since xds and uds are used
  std::vector<StateType> xs_;    ///< vector of states
  std::vector<ControlType> us_;  ///< vector of controls
  std::vector<StateType> xds_;   ///< vector of reference states
  std::vector<ControlType> uds_; ///< vector of reference controls
  Eigen::VectorXd kt_;           ///< Thrust gain
  std::vector<double> ts_;       ///< vector of timestamps
  int look_ahead_index_shift_;   ///< Future time stamp for controller being
                                 /// passed out to account for controller delay
  int max_look_ahead_index_shift_; ///< Future time stamp for controller being
  /// passed out to account for controller delay
  LoopTimer
      loop_timer_; ///< Timer to find the average time taken by a controller
  int control_timer_shift_; ///< How many steps should the control shift by for
                            /// hot starting
  mutable boost::mutex
      copy_mutex_; ///< Synchronize access to states and controls
};
