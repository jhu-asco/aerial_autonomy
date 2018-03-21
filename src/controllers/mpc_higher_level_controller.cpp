#include "aerial_autonomy/controllers/mpc_higher_level_controller.h"

MPCHigherLevelController::MPCHigherLevelController(
    MPCHigherLevelControllerConfig config)
    : config_(config),
      time_grid_(ACADO::Grid(0.0, config_.te(), config_.numsteps())),
      Q_(ACADO::DMatrix(4, 4)),
      ocp_(ACADO::OCP(0.0, config_.te(), config_.numsteps())),
      states_(ACADO::VariablesGrid(14, time_grid_)),
      controls_(ACADO::VariablesGrid(4, time_grid_)) {
  /**
  * Dynamics
  */
  ACADO::DifferentialEquation f; // Differential Equation
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

  /**
  * Cost along trajectory
  */
  Q_.setIdentity();
  Q_(0, 0) = config_.qxx();
  Q_(1, 1) = config_.qyy();
  Q_(2, 2) = config_.qzz();
  Q_(3, 3) = config_.qgg();
  ACADO::DVector offset(4);
  offset.setAll(0.0);
  ACADO::Function eta;
  eta << x4 << y4 << z4 << ga2;

  /**
  * Initialize optimal control problem
  */
  ocp_.subjectTo(f);
  ocp_.minimizeLSQ(Q_, eta, offset);

  /**
  * Set constraints.
  */
  ocp_.subjectTo(-config_.max_yaw() <= ga0 <= config_.max_yaw());
  ocp_.subjectTo(-config_.max_vel() <= x1 <= config_.max_vel());
  ocp_.subjectTo(-config_.max_vel() <= y1 <= config_.max_vel());
  ocp_.subjectTo(-config_.max_vel() <= z1 <= config_.max_vel());
  ocp_.subjectTo(-config_.max_yaw_rate() <= ga1 <= config_.max_yaw_rate());

  // At start
  ocp_.subjectTo(ACADO::AT_START, x1 == 0.0);
  ocp_.subjectTo(ACADO::AT_START, y1 == 0.0);
  ocp_.subjectTo(ACADO::AT_START, z1 == 0.0);
  ocp_.subjectTo(ACADO::AT_START, x2 == 0.0);
  ocp_.subjectTo(ACADO::AT_START, y2 == 0.0);
  ocp_.subjectTo(ACADO::AT_START, z2 == 0.0);
  ocp_.subjectTo(ACADO::AT_START, x3 == 0.0);
  ocp_.subjectTo(ACADO::AT_START, y3 == 0.0);
  ocp_.subjectTo(ACADO::AT_START, z3 == 0.0);
  ocp_.subjectTo(ACADO::AT_START, ga1 == 0.0);

  // At end
  ocp_.subjectTo(ACADO::AT_END, x1 == 0.0);
  ocp_.subjectTo(ACADO::AT_END, y1 == 0.0);
  ocp_.subjectTo(ACADO::AT_END, z1 == 0.0);
  ocp_.subjectTo(ACADO::AT_END, x2 == 0.0);
  ocp_.subjectTo(ACADO::AT_END, y2 == 0.0);
  ocp_.subjectTo(ACADO::AT_END, z2 == 0.0);
  ocp_.subjectTo(ACADO::AT_END, x3 == 0.0);
  ocp_.subjectTo(ACADO::AT_END, y3 == 0.0);
  ocp_.subjectTo(ACADO::AT_END, z3 == 0.0);
  ocp_.subjectTo(ACADO::AT_END, ga1 == 0.0);

  /**
  * Initialize Optimization
  */
  algorithm_.reset(new ACADO::OptimizationAlgorithm(ocp_));
  algorithm_->set(ACADO::MAX_NUM_ITERATIONS,
                  10); // Max iterations before exiting
  algorithm_->set(ACADO::MAX_NUM_QP_ITERATIONS,
                  100); // Max QP iterations before retrying
  algorithm_->set(ACADO::INFEASIBLE_QP_HANDLING,
                  ACADO::IQH_STOP); // Stop if QP problem is infeasible
  algorithm_->set(ACADO::INTEGRATOR_TYPE,
                  ACADO::INT_RK45); // Integrator: Runge-Kutta
  algorithm_->set(ACADO::DISCRETIZATION_TYPE, ACADO::COLLOCATION);
  algorithm_->set(ACADO::HESSIAN_APPROXIMATION, ACADO::GAUSS_NEWTON);
  algorithm_->set(ACADO::KKT_TOLERANCE, 1e-3);
  algorithm_->set(ACADO::PRINTLEVEL, ACADO::LOW);
}

bool MPCHigherLevelController::runImplementation(
    std::tuple<PositionYaw, std::vector<Obstacle>> sensor_data, Position goal,
    std::tuple<Trajectory<QuadFlatOutput>, Trajectory<QuadFlatSpaceControls>>
        &control) {
  /**
  * Set initial position
  */
  PositionYaw initial_position = std::get<0>(sensor_data);

  ocp_.subjectTo(ACADO::AT_START, x0 == initial_position.x);
  ocp_.subjectTo(ACADO::AT_START, y0 == initial_position.y);
  ocp_.subjectTo(ACADO::AT_START, z0 == initial_position.z);
  ocp_.subjectTo(ACADO::AT_START, ga0 == initial_position.yaw);

  ocp_.subjectTo(ACADO::AT_END, x0 == goal.x);
  ocp_.subjectTo(ACADO::AT_END, y0 == goal.y);
  ocp_.subjectTo(ACADO::AT_END, z0 == goal.z);

  ACADO::VariablesGrid initial_states(14, time_grid_);
  ACADO::VariablesGrid initial_controls(4, time_grid_);

  algorithm_->initializeDifferentialStates(initial_states);
  algorithm_->initializeControls(initial_controls);

  /**
  * Solve problem without obstacles to get an
  * initial guess for the trajectory
  */
  algorithm_->solve();

  algorithm_->getDifferentialStates(initial_states);
  algorithm_->getControls(initial_controls);

  /**
  * Add obstacles to OCP
  */
  std::vector<Obstacle> obstacles = std::get<1>(sensor_data);
  for (int i = 0; i < int(obstacles.size()); i++) {
    double ox = obstacles[i].x;
    double oy = obstacles[i].y;
    double oz = obstacles[i].z;
    double ora = obstacles[i].r;
    ocp_.subjectTo((x0 - ox) * (x0 - ox) + (y0 - oy) * (y0 - oy) +
                       (z0 - oz) * (z0 - oz) >=
                   ora * ora);
  }

  /**
  * Reinitialize algorithm with new OCP
  */
  algorithm_.reset(new ACADO::OptimizationAlgorithm(ocp_));
  algorithm_->set(ACADO::MAX_NUM_QP_ITERATIONS, 100);
  algorithm_->set(ACADO::INFEASIBLE_QP_HANDLING, ACADO::IQH_STOP);
  algorithm_->set(ACADO::INTEGRATOR_TYPE, ACADO::INT_RK45);
  algorithm_->set(ACADO::DISCRETIZATION_TYPE, ACADO::COLLOCATION);
  algorithm_->set(ACADO::HESSIAN_APPROXIMATION, ACADO::GAUSS_NEWTON);
  algorithm_->set(ACADO::KKT_TOLERANCE, 1e-3);
  algorithm_->set(ACADO::PRINTLEVEL, ACADO::LOW);

  algorithm_->initializeDifferentialStates(initial_states);
  algorithm_->initializeControls(initial_controls);

  /* Check if optimization is succesful. Return false if not */
  if (algorithm_->solve() != ACADO::SUCCESSFUL_RETURN)
    return false;

  /** Get resulting state and control trajectory */
  algorithm_->getDifferentialStates(states_);
  algorithm_->getDifferentialStates(controls_);

  std::get<0>(final_state_) = PositionYaw(
      states_(config_.numsteps(), 0), states_(config_.numsteps(), 1),
      states_(config_.numsteps(), 2), states_(config_.numsteps(), 12));

  std::get<1>(final_state_) = VelocityYawRate(
      states_(config_.numsteps(), 3), states_(config_.numsteps(), 4),
      states_(config_.numsteps(), 5), states_(config_.numsteps(), 13));

  /** Populate the control variable using the resulting state and control
   * trajectory */
  Trajectory<QuadFlatOutput> output_state_trajectory(
      config_.te() / double(config_.numsteps()));
  Trajectory<QuadFlatSpaceControls> output_control_trajectory(
      config_.te() / double(config_.numsteps()));
  for (int t = 0; t < config_.numsteps(); t++) {
    double time = double(t) / double(config_.numsteps()) * config_.te();
    QuadFlatSpaceControls flat_controls(
        Snap(controls_(t, 0), controls_(t, 1), controls_(t, 3)),
        controls_(t, 4));

    QuadFlatOutput flat_state(
        PositionYaw(states_(t, 0), states_(t, 1), states_(t, 2),
                    states_(t, 12)),
        VelocityYawRate(states_(t, 3), states_(t, 4), states_(t, 5),
                        states_(t, 13)),
        Acceleration(states_(t, 6), states_(t, 7), states_(t, 8)),
        Jerk(states_(t, 9), states_(t, 10), states_(t, 11)));

    output_state_trajectory.setAtTime(flat_state, time);
    output_control_trajectory.setAtTime(flat_controls, time);
  }
  control = std::make_tuple(output_state_trajectory, output_control_trajectory);
  return true;
}

ControllerStatus MPCHigherLevelController::isConvergedImplementation(
    std::tuple<PositionYaw, std::vector<Obstacle>> sensor_data, Position goal) {

  ControllerStatus status = ControllerStatus::Active;
  Position position_diff = goal - std::get<0>(final_state_);
  VelocityYawRate velocity_yawrate_diff = std::get<1>(final_state_);

  double position_tolerance = config_.goal_position_tolerance();
  double velocity_tolerance = config_.goal_velocity_tolerance();

  if (abs(position_diff.x) < position_tolerance &&
      abs(position_diff.y) < position_tolerance &&
      abs(position_diff.z) < position_tolerance &&
      abs(velocity_yawrate_diff.x) < velocity_tolerance &&
      abs(velocity_yawrate_diff.y) < velocity_tolerance &&
      abs(velocity_yawrate_diff.z) < velocity_tolerance &&
      abs(velocity_yawrate_diff.yaw_rate) < velocity_tolerance) {
    status.setStatus(ControllerStatus::Completed);
  }
  return status;
}

bool MPCHigherLevelController::checkTrajectoryFeasibility(
    std::tuple<PositionYaw, std::vector<Obstacle>> sensor_data,
    std::tuple<Trajectory<QuadFlatOutput>, Trajectory<QuadFlatSpaceControls>>
        &control) {
  std::vector<Obstacle> obstacle_list = std::get<1>(sensor_data);
  Trajectory<QuadFlatOutput> state_trajectory = std::get<0>(control);

  if (state_trajectory.trajectory.size() == 0)
    return false;

  for (double t = state_trajectory.ts;
       t <=
       state_trajectory.ts + double(config_.numsteps()) * state_trajectory.dt;
       t += state_trajectory.dt) {
    QuadFlatOutput state = state_trajectory.getAtTime(t);
    if (abs(state.v.x) > config_.max_vel() ||
        abs(state.v.y) > config_.max_vel() ||
        abs(state.v.z) > config_.max_vel() ||
        abs(state.v.yaw_rate) > config_.max_yaw_rate())
      return false;

    for (uint8_t i = 0; i < obstacle_list.size(); i++) {
      double ox = obstacle_list[i].x;
      double oy = obstacle_list[i].y;
      double oz = obstacle_list[i].z;
      double ora = obstacle_list[i].r;
      if (((state.p.x - ox) * (state.p.x - ox) +
           (state.p.y - oy) * (state.p.y - oy) +
           (state.p.z - oz) * (state.p.z - oz)) < ora * ora)
        return false;
    }
  }
  return true;
}
