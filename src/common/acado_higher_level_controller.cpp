#include "aerial_autonomy/common/acado_higher_level_controller.h"

AcadoHigherLevelController::AcadoHigherLevelController(AcadoConfig config)
    : config_(config),
      ocp_(ACADO::OCP(0.0, config_.time_horizon(), config.num_steps())) {
  ACADO::DifferentialEquation f(0.0, config_.time_horizon());
  f << dot(x0) == x1;
  f << dot(y0) == y1;
  f << dot(z0) == z1;

  f << dot(x1) == x2;
  f << dot(y1) == y2;
  f << dot(z1) == z2;

  f << dot(x2) == x3;
  f << dot(y2) == y3;
  f << dot(z2) == z3;

  f << dot(x3) == x4;
  f << dot(y3) == y4;
  f << dot(z3) == z4;

  f << dot(ga0) == ga1;
  f << dot(ga1) == ga2;

  ACADO::DMatrix Q(4, 4);
  Q.setIdentity();
  Q(0, 0) = config_.qxx();
  Q(1, 1) = config_.qyy();
  Q(2, 2) = config_.qzz();
  Q(3, 3) = config_.qgg();
  ACADO::DVector offset(4);
  offset.setAll(0.0);
  ACADO::Function eta;
  eta << x4 << y4 << z4 << ga2;

  ocp_.subjectTo(f);
  ocp_.minimizeLSQ(Q, eta, offset);

  ocp_.subjectTo(-config_.max_yaw() <= ga0 <= config_.max_yaw());
  ocp_.subjectTo(-config_.max_vel() <= x1 <= config_.max_vel());
  ocp_.subjectTo(-config_.max_vel() <= y1 <= config_.max_vel());
  ocp_.subjectTo(-config_.max_vel() <= z1 <= config_.max_vel());
  ocp_.subjectTo(-config_.max_yaw_rate() <= ga1 <= config_.max_yaw_rate());
}

bool AcadoHigherLevelController::solve(
    const std::tuple<QuadFlatOutput, std::vector<Obstacle>> &sensor_data,
    QuadFlatOutput goal, Trajectory<QuadFlatOutput> &control) {
  setInitialandGoalState(std::get<0>(sensor_data), goal);

  algorithm_.reset(new ACADO::OptimizationAlgorithm(ocp_));
  algorithm_->set(ACADO::MAX_NUM_QP_ITERATIONS, 200);
  algorithm_->set(ACADO::INFEASIBLE_QP_HANDLING, ACADO::IQH_STOP);
  algorithm_->set(ACADO::INTEGRATOR_TYPE, ACADO::INT_RK45);
  algorithm_->set(ACADO::DISCRETIZATION_TYPE, ACADO::COLLOCATION);
  algorithm_->set(ACADO::HESSIAN_APPROXIMATION, ACADO::GAUSS_NEWTON);
  algorithm_->set(ACADO::KKT_TOLERANCE, 1e-5);

  ACADO::Grid timeGrid(0.0, config_.time_horizon(), config_.num_steps());
  ACADO::VariablesGrid xi(14, timeGrid);
  ACADO::VariablesGrid ui(4, timeGrid);

  algorithm_->initializeDifferentialStates(xi);
  algorithm_->initializeControls(ui);

  if (!algorithm_->solve())
    return false;

  algorithm_->getDifferentialStates(xi);
  algorithm_->getControls(ui);
  std::vector<Obstacle> obstacle_list = std::get<1>(sensor_data);
  double qr = config_.quad_radius();
  for (int i = 0; i < int(obstacle_list.size()); i++) {
    double ox = obstacle_list[i].x;
    double oy = obstacle_list[i].y;
    double oz = obstacle_list[i].z;
    double ora = obstacle_list[i].r;
    ocp_.subjectTo((x0 - ox) * (x0 - ox) + (y0 - oy) * (y0 - oy) +
                       (z0 - oz) * (z0 - oz) >=
                   (ora + qr) * (ora + qr));
  }

  algorithm_.reset(new ACADO::OptimizationAlgorithm(ocp_));
  algorithm_->set(ACADO::MAX_NUM_QP_ITERATIONS, 200);
  algorithm_->set(ACADO::INFEASIBLE_QP_HANDLING, ACADO::IQH_STOP);
  algorithm_->set(ACADO::INTEGRATOR_TYPE, ACADO::INT_RK45);
  algorithm_->set(ACADO::DISCRETIZATION_TYPE, ACADO::COLLOCATION);
  algorithm_->set(ACADO::HESSIAN_APPROXIMATION, ACADO::GAUSS_NEWTON);
  algorithm_->set(ACADO::KKT_TOLERANCE, 1e-3);

  algorithm_->initializeDifferentialStates(xi);
  algorithm_->initializeControls(ui);

  if (!algorithm_->solve())
    return false;

  algorithm_->getDifferentialStates(xi);
  algorithm_->getControls(ui);

  for (int t = 0; t < config_.num_steps(); t++) {
    QuadFlatOutput state(
        PositionYaw(xi(t, 0), xi(t, 1), xi(t, 2), xi(t, 12)),
        VelocityYawRate(xi(t, 3), xi(t, 4), xi(t, 5), xi(t, 13)),
        Acceleration(xi(t, 6), xi(t, 7), xi(t, 8)),
        Jerk(xi(t, 9), xi(t, 10), xi(t, 11)),
        Snap(ui(t, 0), ui(t, 1), ui(t, 2)), ui(t, 3));
    double time = config_.time_horizon() / double(config_.num_steps()) * t;
    control.setAtTime(state, time);
  }
  return true;
}

void AcadoHigherLevelController::setInitialandGoalState(QuadFlatOutput initial,
                                                        QuadFlatOutput goal) {
  ocp_.subjectTo(ACADO::AT_START, x0 == initial.p.x);
  ocp_.subjectTo(ACADO::AT_START, y0 == initial.p.y);
  ocp_.subjectTo(ACADO::AT_START, z0 == initial.p.z);
  ocp_.subjectTo(ACADO::AT_START, ga0 == initial.p.yaw);

  ocp_.subjectTo(ACADO::AT_START, x1 == 0.0);
  ocp_.subjectTo(ACADO::AT_START, y1 == 0.0);
  ocp_.subjectTo(ACADO::AT_START, z1 == 0.0);
  ocp_.subjectTo(ACADO::AT_START, ga1 == 0.0);

  ocp_.subjectTo(ACADO::AT_START, x2 == 0.0);
  ocp_.subjectTo(ACADO::AT_START, y2 == 0.0);
  ocp_.subjectTo(ACADO::AT_START, z2 == 0.0);

  ocp_.subjectTo(ACADO::AT_START, x3 == 0.0);
  ocp_.subjectTo(ACADO::AT_START, y3 == 0.0);
  ocp_.subjectTo(ACADO::AT_START, z3 == 0.0);

  ocp_.subjectTo(ACADO::AT_END, x0 == goal.p.x);
  ocp_.subjectTo(ACADO::AT_END, y0 == goal.p.y);
  ocp_.subjectTo(ACADO::AT_END, z0 == goal.p.z);

  ocp_.subjectTo(ACADO::AT_END, x1 == 0.0);
  ocp_.subjectTo(ACADO::AT_END, y1 == 0.0);
  ocp_.subjectTo(ACADO::AT_END, z1 == 0.0);
  ocp_.subjectTo(ACADO::AT_END, ga1 == 0.0);

  ocp_.subjectTo(ACADO::AT_END, x2 == 0.0);
  ocp_.subjectTo(ACADO::AT_END, y2 == 0.0);
  ocp_.subjectTo(ACADO::AT_END, z2 == 0.0);

  ocp_.subjectTo(ACADO::AT_END, x3 == 0.0);
  ocp_.subjectTo(ACADO::AT_END, y3 == 0.0);
  ocp_.subjectTo(ACADO::AT_END, z3 == 0.0);
}
