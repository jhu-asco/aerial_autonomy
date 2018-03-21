#pragma once
#include <aerial_autonomy/types/position.h>
/**
* @brief Spherical obstacle
* \todo Add functionality for
* Box and Cylinder
*/

struct Obstacle {
  /**
  * @brief
  */
  Obstacle(double cx, double cy, double cz, double radius)
      : x(cx), y(cy), z(cz), r(radius) {}
  /**
  * @brief Constructor using Position
  */
  Obstacle(Position center, double radius)
      : x(center.x), y(center.y), z(center.z), r(radius) {}
  double x; // X co-ordinate of center
  double y; // Y co-ordinate of center
  double z; // Z co-ordinate of center
  double r; // radius
};
