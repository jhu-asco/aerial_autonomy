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
#include <gcop/lqcost.h>

#include <glog/logging.h>

#include <memory>

class DDPAirmMPCController
    : public AbstractMPCController<
          std::pair<QuadState, ArmState>,
          std::pair<RollPitchYawThrust, std::vector<double>>> {
public:
  using ControlType = std::pair<RollPitchYawThrust, std::vector<double>>;
  using StateType = std::pair<QuadState, ArmState>;
  DDPAirmMPCController(AirmMPCControllerConfig config);
  bool runImplementation(MPCInputs<StateType> sensor_data,
                         ReferenceTrajectory<StateType, ControlType> goal,
                         ControlType &control);

private:
  AirmMPCControllerConfig config_;
  std::unique_ptr<gcop::CasadiSystem<>> sys_;
  std::unique_ptr<gcop::Ddp<Eigen::VectorXd>> ddp_;
  std::unique_ptr<gcop::LqCost<Eigen::VectorXd>> cost_;
  Eigen::VectorXd xf_;
  std::vector<Eigen::VectorXd> xs_;
  std::vector<Eigen::VectorXd> us_;
  std::vector<double> ts_;
};
