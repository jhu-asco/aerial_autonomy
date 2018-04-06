#pragma once
#include "aerial_autonomy/types/acceleration.h"
#include "aerial_autonomy/types/jerk.h"
#include "aerial_autonomy/types/position_yaw.h"
#include "aerial_autonomy/types/roll_pitch_yawrate_thrust.h"
#include "aerial_autonomy/types/snap.h"
#include "aerial_autonomy/types/velocity_yaw_rate.h"
#include <Eigen/Dense>
#include <tf_conversions/tf_eigen.h>
/**
* @brief Store flat output state
* of a quad
*/
struct QuadFlatOutput {
  /**
  * @brief Implicit Constructor
  */
  QuadFlatOutput() {}
  /**
  * @brief Constructor specifying position
  */
  QuadFlatOutput(PositionYaw position_yaw) : p(position_yaw) {}

  /**
  * @brief Explicit Constructor
  */
  QuadFlatOutput(PositionYaw position_yaw, VelocityYawRate velocity_yaw_rate,
                 Acceleration acceleration, Jerk jerk, Snap snap,
                 double yaw_acceleration)
      : p(position_yaw), v(velocity_yaw_rate), a(acceleration), j(jerk),
        s(snap), ga2(yaw_acceleration) {}

  PositionYaw p;     ///< position
  VelocityYawRate v; ///< velocity
  Acceleration a;    ///< Acceleration
  Jerk j;            ///< Jerk
  Snap s;            ///< Snap
  double ga2;        ///< yaw-acceleration

  /**
  * @brief Compare two states
  *
  * @param State to compare with
  * @return True if two states are equal
  */
  bool operator==(const QuadFlatOutput &x) const {
    return (this->p == x.p && this->v == x.v && this->a == x.a &&
            this->j == x.j && this->s == x.s && this->ga2 == x.ga2);
  }

  /**
  * @brief Compare two states
  *
  * @param State to compare with
  * @return True if two states are not equal
  */
  bool operator!=(const QuadFlatOutput &x) const { return !(*this == x); }

  /**
  * @brief Add two states
  * @param x State to add
  * @return Sum of two states
  */
  QuadFlatOutput operator+(const QuadFlatOutput &x) const {
    return QuadFlatOutput(this->p + x.p, this->v + x.v, this->a + x.a,
                          this->j + x.j, this->s + x.s, this->ga2 + x.ga2);
  }
  /**
  * @brief Subtract two states
  * @param x State to subtract
  * @return Difference of two states
  */
  QuadFlatOutput operator-(const QuadFlatOutput &x) const {
    return QuadFlatOutput(this->p - x.p, this->v - x.v, this->a - x.a,
                          this->j - x.j, this->s - x.s, this->ga2 - x.ga2);
  }
  /*
  * @brief Get rpyt from flat outputs (t is thrust scaled by kt)
  */
  RollPitchYawRateThrust getRPYT() {
    RollPitchYawRateThrust rpyt;
    double T = sqrt(a.x * a.x + a.y * a.y + (a.z + 9.81) * (a.z + 9.81));
    rpyt.t = T;
    rpyt.y = v.yaw_rate;
    double yaw = p.yaw;
    // Acceleration in gravity aligned yaw-compensated frame
    Eigen::Vector3d rot_acc;
    rot_acc[0] = a.x * cos(yaw) + a.y * sin(yaw);
    rot_acc[1] = -a.x * sin(yaw) + a.y * cos(yaw);
    rot_acc[2] = a.z + 9.81;
    rot_acc = rot_acc / (rot_acc.norm());
    rpyt.r = -asin(rot_acc[1]);
    rpyt.p = atan2(rot_acc[0], rot_acc[2]);
    return rpyt;
  }
};
