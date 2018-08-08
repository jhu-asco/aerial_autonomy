#pragma once
#include "aerial_autonomy/controllers/ddp_casadi_mpc_controller.h"
#include "airm_mpc_controller_config.pb.h"

/**
* @brief DDP based MPC controller for Aerial manipulation system
*
* Performs DDP on a Quadrotor model with a 2DOF arm attached. Uses
* GCOP library for defining the system and performing optimization
*/
class DDPAirmMPCController : public DDPCasadiMPCController {
public:
  /**
  * @brief Constructor
  *
  * @param config MPC controller config
  * @param controller_duration The time step for calling controller
  */
  DDPAirmMPCController(AirmMPCControllerConfig config,
                       std::chrono::duration<double> controller_duration);

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
  * @brief Load arm model parameters from text file
  *
  * @param kp_ja Second order kp gain on joint angles
  * @param kd_ja Second order kd gain on joint angles
  * @param folder_path The folder where text file is present
  */
  void loadArmParameters(Eigen::Vector2d &kp_ja, Eigen::Vector2d &kd_ja,
                         std::string folder_path);

  /**
  * @brief Overwrite the MPC Config
  *
  * @param config MPC controller config
  */
  void setConfig(AirmMPCControllerConfig config);

protected:
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
  virtual ControlType stationaryControl();

  virtual void outputControl(ControlType &control);

  virtual void logData(MPCInputs<StateType> &sensor_data, ControlType &control);

private:
  AirmMPCControllerConfig config_; ///< MPC controller config
};
