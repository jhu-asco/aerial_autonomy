#pragma once

#include "aerial_autonomy/types/position_yaw.h"
#include <tf_conversions/tf_eigen.h>

#include "position_yaw.pb.h"

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
}
