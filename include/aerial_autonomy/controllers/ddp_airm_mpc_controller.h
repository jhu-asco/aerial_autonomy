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

class DDPAirmMPCController
    : public AbstractMPCController<Eigen::VectorXd, Eigen::VectorXd> {
public:
  using ControlType = Eigen::VectorXd;
  using StateType = Eigen::VectorXd;
  using GoalType = ReferenceTrajectoryPtr<StateType, ControlType>;
  DDPAirmMPCController(AirmMPCControllerConfig config,
                       double controller_duration);
  void resetControls();

protected:
  bool runImplementation(MPCInputs<StateType> sensor_data, GoalType goal,
                         ControlType &control);
  ControllerStatus isConvergedImplementation(MPCInputs<StateType> sensor_data,
                                             GoalType);

private:
  AirmMPCControllerConfig config_;
  std::unique_ptr<gcop::CasadiSystem<>> sys_;
  std::unique_ptr<gcop::Ddp<Eigen::VectorXd>> ddp_;
  std::unique_ptr<gcop::LqCost<Eigen::VectorXd>> cost_;
  Eigen::VectorXd xf_;
  std::vector<Eigen::VectorXd> xs_;
  std::vector<Eigen::VectorXd> us_;
  std::vector<Eigen::VectorXd> xds_;
  std::vector<Eigen::VectorXd> uds_;
  Eigen::VectorXd kt_;
  std::vector<double> ts_;
  int look_ahead_index_shift_;
  LoopTimer loop_timer_;
  int control_timer_shift_;
};
