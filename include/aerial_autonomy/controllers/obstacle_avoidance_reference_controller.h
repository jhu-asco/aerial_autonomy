#pragma once
#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/controllers/base_controller.h"
#include "aerial_autonomy/types/minimum_snap_reference_trajectory.h"
#include "aerial_autonomy/types/particle_state.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/reference_trajectory.h"
#include "aerial_autonomy/types/snap.h"
#include "multiple_spherical_obstacles.pb.h"

#include "ceres/ceres.h"
#include <Eigen/Dense>

class ObstacleAvoidanceReferenceController
    : public Controller<
          // DataType: start pos and obstacles
          std::pair<PositionYaw, MultipleSphericalObstacles>,
          // GoalTYpe: Goal position
          PositionYaw,
          // ControlType: ReferenceTrajectory
          ReferenceTrajectoryPtr<ParticleState, Snap>> {
public:
  /**
   * @brief constructor
   *
   * @param config particle reference config
   */
  ObstacleAvoidanceReferenceController(int num_way_points)
      : num_way_points_(num_way_points), num_segments_(num_way_points_ + 1) {}

protected:
  /**
   * @brief generate reference trajectory based on sensor data, goal
   *
   * @param sensor_data current position, object transform in quad frame
   * @param goal Goal in object frame
   * @param control Reference in global frame
   *
   * @return true if able to generate the trajectory
   */
  bool runImplementation(
      std::pair<PositionYaw, MultipleSphericalObstacles> sensor_data,
      PositionYaw goal, ReferenceTrajectoryPtr<ParticleState, Snap> control);

  ControllerStatus isConvergedImplementation(
      std::pair<PositionYaw, MultipleSphericalObstacles> sensor_data,
      PositionYaw goal);

private:
  int num_way_points_;
  const int num_segments_;
};

// Cost Functor
struct CostFunctor {
  // Constructor
  // TODO: Can struct inside class object call member variables?
  // TODO: convert sensor_data. How about start and goal?
  CostFunctor(std::pair<PositionYaw, MultipleSphericalObstacles> sensor_data,
              PositionYaw goal)
      : sensor_data_(sensor_data), start_(std::get<0>(sensor_data_)),
        goal_(goal) {
    // proto: repeated spherical obstacle
    MultipleSphericalObstacles obstacles = std::get<1>(sensor_data_);
    Eigen::Array3d obstacle_pos;
    for (int i = 0; i < obstacles.spherical_obstacle_size(); i++) {
      // proto positionyaw & radius
      auto obstacle = obstacles.spherical_obstacle(i);
      PositionYaw obs_pos =
          conversions::protoPositionYawToPositionYaw(obstacle.position());
      obstacle_pos(0, i) = obs_pos.position().x;
      obstacle_pos(1, i) = obs_pos.position().y;
      obstacle_pos(2, i) = obs_pos.position().z;
      safety_threshold_(i) = obstacle.radius();
    }
    obstcle_pos_ = obstacle_pos;
  }

  // get Discrete trajectory for position (x,y,z)
  Eigen::Array3Xd getDiscreteTraj(MinimumSnapReferenceTrajectory &trajectory,
                                  double total_time,
                                  std::size_t discretization) const {
    // std::size_t discretization = 100;
    Eigen::MatrixXd discrete_poses(3, discretization);
    for (std::size_t i = 0; i < discretization; i++) {
      double t = (double)i / discretization * total_time;
      std::pair<ParticleState, Snap> state = trajectory.atTime(t);
      // particle state
      ParticleState particle_state = std::get<0>(state);
      // Position
      discrete_poses.col(i) = conversions::toEigen(particle_state.p);
    }
    return discrete_poses.array();
  }

  bool operator()(const double *way_points, const double *time_intervals,
                  double *residuals) const {
    // make path
    // Eigen::MatrixXd path = convertArrayToEigenMat(way_points);
    // # of segments
    int size = sizeof(time_intervals) / sizeof(time_intervals[0]);
    int size_way_points = sizeof(way_points) / sizeof(way_points[0]);
    int dimension = size_way_points / (size - 1);
    Eigen::MatrixXd path(size + 1, dimension);
    path(0, 0) = start_.position().x;
    path(0, 1) = start_.position().y;
    path(0, 2) = start_.position().z;
    for (int i = 1; i < size; i++) {
      path(i, 0) = way_points[3 * i];
      path(i, 1) = way_points[3 * i + 1];
      path(i, 2) = way_points[3 * i + 2];
    }
    path(size, 0) = goal_.position().x;
    path(size, 1) = goal_.position().y;
    path(size, 2) = goal_.position().z;
    // path = [start, path, goal];
    // tau_vec
    // Eigen::VectorXd tau_vec = convertArrayToEigenVec(time_intervals);
    Eigen::VectorXd tau_vec(size);
    for (int i = 0; i < size; i++) {
      tau_vec(i) = time_intervals[i];
    }
    double total_time = tau_vec.sum();
    // trajectory object
    MinimumSnapReferenceTrajectory trajectory(4, tau_vec, path);
    // P = trajectory.getP(); // This may not need.
    // getPos
    // TODO Vectorize below to get discrete trajectory
    // discrete_poses.row(0) -= obstcle_pos_(0);
    // discrete_poses.row(1) -= obstcle_pos_(1);
    // discrete_poses.row(2) -= obstcle_pos_(2);
    Eigen::Array3Xd discrete_poses =
        getDiscreteTraj(trajectory, total_time, 100);
    double J_obs = 0.0;
    for (int i = 0; i < obstcle_pos_.cols(); i++) {
      Eigen::Array3Xd discrete_poses_tmp =
          discrete_poses.colwise() - obstcle_pos_.col(i);

      // Eigen::ArrayXd distances = safety_threshold_(i) -
      // discrete_poses_tmp.colwise().norm();
      Eigen::ArrayXd distances =
          safety_threshold_(i) -
          discrete_poses_tmp.square().colwise().sum().sqrt();
      // replace all negative distance with zero
      distances = (distances < 0).select(0, distances);
      J_obs += distances.square().sum();
    }

    // Cost
    double J_traj = trajectory.getCost();
    double J_time = total_time;

    residuals[0] = J_traj + J_obs + J_time;
    return true;
  }

  std::pair<PositionYaw, MultipleSphericalObstacles> sensor_data_;
  PositionYaw start_;
  PositionYaw goal_;
  Eigen::Array3d obstcle_pos_;
  Eigen::VectorXd safety_threshold_;
};
