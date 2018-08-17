#pragma once

#include "aerial_autonomy/types/acceleration.h"
#include "aerial_autonomy/types/jerk.h"
#include "aerial_autonomy/types/position.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/velocity.h"
#include "aerial_autonomy/types/waypoint.h"
#include "parsernode/common.h"
#include <armadillo>

#include "position_yaw.pb.h"
#include "transform.pb.h"

namespace conversions {

/**
 * @brief Convert an Eigen::Matrix4d to a tf::Transform
 * @param e Input Eigen::Matrix4d
 * @param tf Output tf::Transform
 */
void transformMatrix4dToTf(const Eigen::Matrix4d &e, tf::Transform &tf);

/**
* @brief Convert transform into roll pitch yaw in body zyx format
*
* @param tf Transform from which rpy is extracted
*
* @returns rpy roll pitch yaw vector
*/
Eigen::Vector3d transformTfToRPY(const tf::Transform &tf);

/**
 * @brief Convert roll, pitch, yaw Euler angles to a tf::Transform
 * @param r Roll
 * @param p Pitch
 * @param y Yaw
 * @param tf Output tf::Transform
 */
void transformRPYToTf(double r, double p, double y, tf::Transform &tf);

/**
 * @brief Convert PositionYaw to tf::Transform
 * @param p PositionYaw to convert
 * @param tf The equivalent tf::Transform
 */
void positionYawToTf(const PositionYaw &p, tf::Transform &tf);

/**
 * @brief Convert tf transform to position yaw
 *
 * @param p positon yaw output
 * @param tf input tf transform
 */
void tfToPositionYaw(PositionYaw &p, const tf::Transform &tf);

/**
 * @brief Convert T to Eigen::Vector3d
 * @param p Position to convert
 * @return The equivalent Eigen::Vector3d
 */
template <class T> Eigen::Vector3d toEigen(const T &p) {
  return Eigen::Vector3d(p.x, p.y, p.z);
}

/**
 * @brief Convert an Eigen::MatrixXd to an arma::mat
 * @param m Eigen matrix
 * @return arma matrix
 */
arma::mat eigenToArma(const Eigen::MatrixXd &m);

/**
 * @brief Convert an arma::mat to a Eigen::MatrixXd
 * @param m arma matrix
 * @return Eigen matrix
 */
Eigen::MatrixXd armaToEigen(const arma::mat &m);

/**
 * @brief Convert config::Transform to tf::Transform
 * @param p config::Transform to convert
 * @param tf The equivalent tf::Transform
 */
tf::Transform protoTransformToTf(const config::Transform &tf);

/**
* @brief Convert a proto PositionYaw to a PositionYaw
* @param p Proto PositionYaw to convert
* @return Converted PositionYaw
*/
PositionYaw protoPositionYawToPositionYaw(config::PositionYaw p);

/**
* @brief Convert a list proto PositionYaw to a list of PositionYaw
* @param p Proto PositionYaw list to convert
* @return Converted PositionYaw list
*/
template <class T>
std::vector<PositionYaw> protoPositionYawsToPositionYaws(const T &proto_ps) {
  std::vector<PositionYaw> ps;
  for (auto proto_p : proto_ps) {
    ps.push_back(protoPositionYawToPositionYaw(proto_p));
  }
  return ps;
}

/**
* @brief Convert a list proto Transform to a list of tf::Transform
* @param p Proto Transform list to convert
* @return Converted tf::Transform list
*/
template <class T>
std::vector<tf::Transform> protoTransformsToTfs(const T &proto_tfs) {
  std::vector<tf::Transform> tfs;
  for (auto proto_tf : proto_tfs) {
    tfs.push_back(protoTransformToTf(proto_tf));
  }
  return tfs;
}

/**
* @brief Convert a list to an Eigen vector
* @param p Proto list to convert
* @return Converted Eigen vector
*/
template <class T> Eigen::VectorXd vectorProtoToEigen(const T &xs) {
  int N = xs.size();
  Eigen::VectorXd v(N);
  for (int i = 0; i < N; ++i) {
    v[i] = xs.Get(i);
  }
  return v;
}
/**
  * @brief create a waypoint reference trajectory from a goal point and joint
  * angles
  *
  * @param goal Goal position and yaw
  * @param desired_joint_angle_1 Desired joint angle for first joint
  * @param desired_joint_angle_2 Desired joint angle for second joint
  *
  * @return Waypoint Reference trajectory
  */
std::shared_ptr<Waypoint<Eigen::VectorXd, Eigen::VectorXd>>
createWaypoint(PositionYaw goal, double desired_joint_angle_1,
               double desired_joint_angle_2);

/**
  * @brief create a waypoint reference trajectory from a goal point
  * for a quad
  *
  * @param goal Goal position and yaw
  *
  * @return Waypoint Reference trajectory
  */
std::shared_ptr<Waypoint<Eigen::VectorXd, Eigen::VectorXd>>
createWaypoint(PositionYaw goal);
/**
* @brief Convert a Eigen::VectorXd to std::vector<double>
* @param vec_eigen Eigen::VectorXd
* @return std::vector<double>
*/
std::vector<double> vectorEigenToStd(const Eigen::VectorXd &vec_eigen);

/**
 * @brief Map acceleration vector to roll, pitch
 *
 * @param yaw Current yaw
 * @param acceleration_vector Acceleration vector
 *
 * @return  roll, pitch as a pair
 */
std::pair<double, double>
accelerationToRollPitch(double yaw, Eigen::Vector3d acceleration_vector);

/**
 * @brief get the pose of quadrotor as a tf transform
 *
 * @param data Quad data
 *
 * @return current pose of quadrotor
 */
tf::Transform getPose(const parsernode::common::quaddata &data);

/**
 * @brief Compute euler angle rates from body angular velocities
 *
 * @param omega Body angular velocities
 * @param rpy Euler angles
 * @param max_pitch Maximum pitch should be less than pi/2 to avoid
 *                  singularities
 *
 * @return Euler angle rates
 */
Eigen::Vector3d omegaToRpyDot(const Eigen::Vector3d &omega,
                              const Eigen::Vector3d &rpy,
                              const double max_pitch = 0.9 * (M_PI / 2.0));
}
