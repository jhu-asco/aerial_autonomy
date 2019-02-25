#include "aerial_autonomy/types/adaptive_polynomial_reference_trajectory.h"
#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/common/math.h"
#include <glog/logging.h>

AdaptivePolynomialReferenceTrajectory::AdaptivePolynomialReferenceTrajectory(
    PositionYaw goal_state, PositionYaw start_state,
    PolynomialReferenceConfig config): 
    wrapped_traj_(goal_state, start_state, config)
{}

Eigen::MatrixXd
AdaptivePolynomialReferenceTrajectory::findBasisMatrix(double t, int degree,
                                               int dimensions) const {
  return wrapped_traj_.findBasisMatrix(t,degree,dimensions);
}

Eigen::Vector3d AdaptivePolynomialReferenceTrajectory::getNoise(double t, double a,
                                                        double nu) const {
  return wrapped_traj_.getNoise(t,a,nu);
}

std::pair<ParticleStateYaw, Snap>
AdaptivePolynomialReferenceTrajectory::atTime(double t) const {
  std::pair<Eigen::VectorXd,Eigen::VectorXd> wrapped_result = wrapped_traj_.atTime(t);
  //Convert to particlestateyaw and snap
  Eigen::VectorXd state = wrapped_result.first;
  Eigen::VectorXd control = wrapped_result.second;
  Position p(state[0],state[1],state[2]);
  double roll = state[3];
  double pitch = state[4];
  double yaw = state[5];
  Velocity v(state[6],state[7],state[8]);
  Eigen::Matrix3d R;
  conversions::transformRPYToMatrix3d(roll,pitch,yaw,R);
  Eigen::Vector3d a_vec(0,0,control[0]);
  a_vec = R*a_vec;
  Acceleration a(a_vec[0],a_vec[1],a_vec[2]);
  Jerk j;//Leave blank
  Snap s;//Leave blank
  ParticleStateYaw particle(p,v,a,j,yaw);
  return std::make_pair(particle,s);
}

ParticleStateYaw AdaptivePolynomialReferenceTrajectory::goal(double) {
  Eigen::VectorXd goal_state = wrapped_traj_.goal(0);
  //Convert to particlestateyaw and snap;
  Position p(goal_state[0],goal_state[1],goal_state[2]);
  double yaw = goal_state[5];
  Velocity v(goal_state[6],goal_state[7],goal_state[8]);
  Acceleration a;//Zero at the goal
  Jerk j;//Leave blank
  ParticleStateYaw particle(p,v,a,j,yaw);
  return particle;
}
