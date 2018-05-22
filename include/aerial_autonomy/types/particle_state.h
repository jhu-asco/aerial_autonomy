#pragma once

#include "aerial_autonomy/types/acceleration.h"
#include "aerial_autonomy/types/jerk.h"
#include "aerial_autonomy/types/position.h"
#include "aerial_autonomy/types/velocity.h"

/**
* @brief State of 3D 4th order particle system
*/
struct ParticleState {
  /**
  * @brief Position
  */
  Position p;
  /**
  * @brief Velocity
  */
  Velocity v;
  /**
  * @brief Acceleration
  */
  Acceleration a;
  /**
  * @brief Jerk
  */
  Jerk j;

  /**
  * @brief Constructor
  */
  ParticleState() : p(0, 0, 0), v(0, 0, 0), a(0, 0, 0), j(0, 0, 0) {}

  /**
  * @brief Constructor
  * @param p Position
  * @param v Velocity
  * @param a Acceleration
  * @param j Jerk
  */
  ParticleState(Position p, Velocity v, Acceleration a, Jerk j)
      : p(p), v(v), a(a), j(j) {}

  /**
  * @brief Multiply times a scalar
  * @param m Multiplier
  * @return Scaled State
  */
  ParticleState operator*(const double &m) const {
    return ParticleState(this->p * m, this->v * m, this->a * m, this->j * m);
  }

  /**
  * @brief Add particle states
  * @param p ParticleState to add
  * @return Summed ParticleStates
  */
  ParticleState operator+(const ParticleState &p) const {
    return ParticleState(this->p + p.p, this->v + p.v, this->a + p.a,
                         this->j + p.j);
  }
};
