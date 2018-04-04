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

  ocp_.subjectTo(f);

  ocp_.subjectTo(-config_.max_yaw() <= ga0 <= config_.max_yaw());
  ocp_.subjectTo(-config_.max_vel() <= x1 <= config_.max_vel());
  ocp_.subjectTo(-config_.max_vel() <= y1 <= config_.max_vel());
  ocp_.subjectTo(-config_.max_vel() <= z1 <= config_.max_vel());
  ocp_.subjectTo(-config_.max_yaw_rate() <= ga1 <= config_.max_yaw_rate());
}

bool AcadoHigherLevelController::checkTrajectoryFeasibility(
    const Trajectory<QuadFlatOutput> &trajectory,
    const std::vector<Obstacle> &obstacle_list, QuadFlatOutput goal) {
  double qr = config_.quad_radius();
  for (int t = 0; t < int(trajectory.trajectory.size() - 2); t++) {
    double x = trajectory.trajectory[t].p.x;
    double y = trajectory.trajectory[t].p.y;
    double z = trajectory.trajectory[t].p.z;
    double yaw = trajectory.trajectory[t].p.yaw;

    double vx = trajectory.trajectory[t].v.x;
    double vy = trajectory.trajectory[t].v.y;
    double vz = trajectory.trajectory[t].v.z;
    double yaw_rate = trajectory.trajectory[t].v.yaw_rate;

    if (abs(vx) > config_.max_vel() || abs(vy) > config_.max_vel() ||
        abs(vz) > config_.max_vel() || abs(yaw_rate) > config_.max_yaw_rate() ||
        abs(yaw) > config_.max_yaw())
      return false;

    for (int o = 0; o < int(obstacle_list.size()); o++) {
      double ox = obstacle_list[o].x;
      double oy = obstacle_list[o].y;
      double oz = obstacle_list[o].z;
      double ora = obstacle_list[o].r;

      if (((x - ox) * (x - ox) + (y - oy) * (y - oy) + (z - oz) * (z - oz)) <
          (qr + ora) * (qr + ora))
        return false;
    }
  }
  double e = config_.tolerance();
  QuadFlatOutput error = goal - final_state_;
  if (config_.terminal_constraint())
    if (abs(error.p.x) > e || abs(error.p.y) > e || abs(error.p.z) > e ||
        abs(error.p.yaw) > e || abs(error.v.x) > e || abs(error.v.y) > e ||
        abs(error.v.z) > e || abs(error.v.yaw_rate) > e || abs(error.a.x) > e ||
        abs(error.a.y) > e || abs(error.a.z) > e || abs(error.j.x) > e ||
        abs(error.j.y) > e || abs(error.j.z) > e) {
      return false;
    }

  return true;
}

bool AcadoHigherLevelController::solve(
    const std::tuple<QuadFlatOutput, std::vector<Obstacle>> &sensor_data,
    Trajectory<QuadFlatOutput> &goal, Trajectory<QuadFlatOutput> &control) {
  setInitialAndGoalConditions(std::get<0>(sensor_data), goal);
  std::unique_ptr<ACADO::OptimizationAlgorithm>
      algorithm_; ///< Optimization Algorithm
  algorithm_.reset(new ACADO::OptimizationAlgorithm(ocp_));
  algorithm_->set(ACADO::MAX_NUM_QP_ITERATIONS, 200);
  algorithm_->set(ACADO::INFEASIBLE_QP_HANDLING, ACADO::IQH_STOP);
  algorithm_->set(ACADO::INTEGRATOR_TYPE, ACADO::INT_RK45);
  algorithm_->set(ACADO::DISCRETIZATION_TYPE, ACADO::COLLOCATION);
  algorithm_->set(ACADO::HESSIAN_APPROXIMATION, ACADO::GAUSS_NEWTON);
  algorithm_->set(ACADO::KKT_TOLERANCE, 1e-5);

  ACADO::Grid timeGrid(0.0, config_.time_horizon(), config_.num_steps() + 1);
  ACADO::VariablesGrid xi(14, timeGrid);
  ACADO::VariablesGrid ui(4, timeGrid);

  algorithm_->initializeDifferentialStates(xi);
  algorithm_->initializeControls(ui);

  if (!algorithm_->solve())
    return false;

  algorithm_->getDifferentialStates(xi);
  algorithm_->getControls(ui);
  std::vector<Obstacle> obstacle_list = std::get<1>(sensor_data);
  if (obstacle_list.size() > 0) {
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
  }

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
  int t = config_.num_steps();
  final_state_ = QuadFlatOutput(
      PositionYaw(xi(t, 0), xi(t, 1), xi(t, 2), xi(t, 12)),
      VelocityYawRate(xi(t, 3), xi(t, 4), xi(t, 5), xi(t, 13)),
      Acceleration(xi(t, 6), xi(t, 7), xi(t, 8)),
      Jerk(xi(t, 9), xi(t, 10), xi(t, 11)), Snap(0.0, 0.0, 0.0), 0.0);
  return true;
}

void AcadoHigherLevelController::setInitialAndGoalConditions(
    QuadFlatOutput initial, Trajectory<QuadFlatOutput> &goal) {
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

  if (config_.terminal_constraint()) {
    double h = goal.horizon();
    QuadFlatOutput final_state = goal.getAtTime(h);
    ocp_.subjectTo(ACADO::AT_END, x0 == final_state.p.x);
    ocp_.subjectTo(ACADO::AT_END, y0 == final_state.p.y);
    ocp_.subjectTo(ACADO::AT_END, z0 == final_state.p.z);

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

    /* Add cost only for control effort */
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
    ocp_.minimizeLSQ(Q, eta, offset);
  } else {
    // \todo Add cost for deviation from reference trajectory
    LOG(WARNING)
        << "Functionality for taking reference trajectory not implemented yet";
  }
}
