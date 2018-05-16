#pragma once
#include <tf/tf.h>
/**
* @brief Generic constraint and its parameters
*/
struct Constraint {
  /**
  * @brief Type of constraint
  */
  enum struct ConstraintType {
    /**
    * @brief A spherical obstacle where scale[0] refers to radius
    */
    Sphere,
    /**
    * @brief The z axis of cylinder is assumed along the z axis of transform.
    * The radius of cylinder is given by scale[0], scale[1]. The length of
    * the cylinder is given by 2*scale[2]. The center of the cylinder is
    * the origin of the transform.
    */
    Cylinder,
    /**
    * @brief The pose of the ellipsoid is given by the transform.
    * The semi-major axes are given by the vector scale
    */
    Ellipsoid,
    /**
    * @brief The box pose is given by the transform. The center of the box
    * is given by the transform origin. The half-lengths along each axes
    * are given by the scale vector.
    */
    Box
  } constraint_type;
  /**
  * @brief the pose of the obstacle/constraint
  * For cylinders, the major axis is assumed to be along the z axis
  */
  tf::Transform transform;
  /**
  * @brief The length of the obstacle along the body x, y, z axes
  */
  tf::Vector3 scale;
};
