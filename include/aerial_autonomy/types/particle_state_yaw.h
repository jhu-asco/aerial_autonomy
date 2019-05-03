#pragma once

#include "aerial_autonomy/types/particle_state.h"

/**
* @brief State of 3D 4th order particle system with yaw added
*/
struct ParticleStateYaw : public ParticleState {

  //Yaw
  double yaw;

  ParticleStateYaw() : ParticleState(), yaw(0) {}

  /**
  * @brief Constructor
  * @param p Position
  * @param v Velocity
  * @param a Acceleration
  * @param j Jerk
  * @param y double yaw
  */
  ParticleStateYaw(Position p, Velocity v, Acceleration a, Jerk j, double y)
      : ParticleState(p,v,a,j), yaw(y) {}

};
