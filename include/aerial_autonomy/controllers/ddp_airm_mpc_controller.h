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
///\todo Add initialize part to connector which basically runs the controller
/// once but does not send
/// commands to quad

class DDPAirmMPCController
    : public AbstractMPCController<Eigen::VectorXd, Eigen::VectorXd> {
public:
  using ControlType = Eigen::VectorXd;
  using StateType = Eigen::VectorXd;
  using GoalType = ReferenceTrajectoryPtr<StateType, ControlType>;
  DDPAirmMPCController(AirmMPCControllerConfig config,
                       std::chrono::duration<double> controller_duration,
                       bool use_code_generation = true);
  void resetControls();

  void getTrajectory(std::vector<StateType> &xs,
                     std::vector<ControlType> &us) const;

  void getDesiredTrajectory(std::vector<StateType> &xds,
                            std::vector<ControlType> &uds) const;

  void loadQuadParameters(Eigen::Vector3d &kp_rpy, Eigen::Vector3d &kd_rpy,
                          Eigen::VectorXd &p, std::string folder_path);

  void loadArmParameters(Eigen::Vector2d &kp_ja, Eigen::Vector2d &kd_ja,
                         std::string folder_path);

  void rotateControls(int shift_length);

  double getLoopTime() { return loop_timer_.average_loop_period(); }

  void setConfig(AirmMPCControllerConfig config);

  double getMPCCost() { return ddp_->J; }

protected:
  bool runImplementation(MPCInputs<StateType> sensor_data, GoalType goal,
                         ControlType &control);
  ControllerStatus isConvergedImplementation(MPCInputs<StateType> sensor_data,
                                             GoalType goal);

private:
  AirmMPCControllerConfig config_;
  std::unique_ptr<gcop::CasadiSystem<>> sys_;
  std::unique_ptr<gcop::Ddp<Eigen::VectorXd>> ddp_;
  std::unique_ptr<gcop::LqCost<Eigen::VectorXd>> cost_;
  Eigen::VectorXd xf_; ///< Not being used since xds and uds are used
  std::vector<StateType> xs_;
  std::vector<ControlType> us_;
  std::vector<StateType> xds_;
  std::vector<ControlType> uds_;
  Eigen::VectorXd kt_;
  std::vector<double> ts_;
  int look_ahead_index_shift_;
  LoopTimer loop_timer_;
  int control_timer_shift_;
  mutable boost::mutex copy_mutex_; ///< Synchronize access to data
};
