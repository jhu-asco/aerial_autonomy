#pragma once

#include "aerial_autonomy/types/acceleration.h"
#include "aerial_autonomy/types/jerk.h"
#include "aerial_autonomy/types/position.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/velocity.h"
#include <armadillo>
#include <tf_conversions/tf_eigen.h>

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
  Eigen::VectorXd v;
  for (auto x : xs) {
    v << x;
  }
  return v;
}
}
