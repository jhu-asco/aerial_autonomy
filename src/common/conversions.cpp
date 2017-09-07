#include "aerial_autonomy/common/conversions.h"

/**
 * @brief Namespace for converting eign to tf matrices
 */
namespace conversions {
// \todo Matt add tests for transformMatrix4dToTf and transformRPYToTf
void transformMatrix4dToTf(const Eigen::Matrix4d &e, tf::Transform &tf) {
  Eigen::Affine3d e_affine;
  e_affine.matrix() = e;
  tf::transformEigenToTF(e_affine, tf);
}

void transformRPYToTf(double r, double p, double y, tf::Transform &tf) {
  tf.setIdentity();
  tf.setRotation(tf::createQuaternionFromRPY(r, p, y));
}

void positionYawToTf(const PositionYaw &p, tf::Transform &tf) {
  tf.setRotation(tf::createQuaternionFromRPY(0, 0, p.yaw));
  tf.setOrigin(tf::Vector3(p.x, p.y, p.z));
}

PositionYaw protoPositionYawToPositionYaw(config::PositionYaw p) {
  return PositionYaw(p.position().x(), p.position().y(), p.position().z(),
                     p.yaw());
}
}
