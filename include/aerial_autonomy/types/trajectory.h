#pragma once
/**
* @brief Trajectory of specified datatype
*/
template <class T> struct Trajectory {
  /**
  * @brief Implicit Constructor
  */
  Trajectory() : Trajectory(0.1) {}
  /**
  * @brief Constructor
  *
  * @param timestep Timestep of the trajectory
  * @param start_time Start time of trajectory
  */
  Trajectory(double timestep, double start_time = 0.0)
      : dt(timestep), ts(start_time) {}
  std::vector<T> trajectory; // array of states
  double dt;                 // timestep
  double ts;                 // start time
                             /**
                             * @brief returns horizon length in terms of time
                             */
  double horizon() { return (ts + double(trajectory.size()) * dt); }
  /**
  * @brief set Value at given time
  * Rounds it up to nearest time less than or
  * equal to t according to dt.
  * If current horizon is smaller resizes the array
  */
  void setAtTime(const T &data, double t) {
    int i = int((t - ts) / dt);
    if (i >= int(trajectory.size()))
      trajectory.resize(i + 1);
    trajectory[i] = data;
  }
  /**
  * @brief get value at given time
  *
  * returns initial state if queried for
  * time before ts
  * returns final state if queried for
  * time beyond horizon
  */
  T getAtTime(double t) {
    int i = int((t - ts) / dt);
    if (i < 0)
      return trajectory[0];
    if (i >= int(trajectory.size()))
      return trajectory[trajectory.size() - 1];
    return trajectory[i];
  }
  /**
  * @brief get the piece of trajectory starting and ending
  * at certain times
  *
  * @param start Start time
  * @param end End time
  */
  Trajectory<T> getPiece(double start, double end) {
    Trajectory<T> piece(dt, start);
    for (double time = start; time <= end; time += dt) {
      piece.setAtTime(getAtTime(time), time);
    }
    return piece;
  }
};