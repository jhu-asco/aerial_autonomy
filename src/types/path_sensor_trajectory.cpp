#include "aerial_autonomy/types/path_sensor_trajectory.h"
#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/common/math.h"
#include <glog/logging.h>
#include <chrono>

PathSensorTrajectory::PathSensorTrajectory(
    SensorPtr<PathReturnT> path_sensor,
    std::chrono::time_point<std::chrono::high_resolution_clock> goal_time)
    : sensor_(path_sensor), goal_time_(goal_time) {}

std::pair<Eigen::VectorXd, Eigen::VectorXd>
PathSensorTrajectory::atTime(double t) const {
  //Unpack sensor data
  if (!sensor_) {
    // THIS SHOULD NEVER HAPPEN!!
    std::cout << "Sensor is null" << std::endl;
    return std::make_pair(Eigen::VectorXd(), Eigen::VectorXd());
  }
  PathReturnT sensor_data = sensor_->getSensorData();
  double path_start_time = std::get<0>(sensor_data);
  double goal_time_secs =
      std::chrono::duration<double>(goal_time_.time_since_epoch()).count();
  double path_duration = std::get<1>(sensor_data);
  std::vector<PathPointT> path_data = std::get<2>(sensor_data);
  //Calculate index
  //double final_time = path_start_time + path_duration;
  int N = path_data.size();
  double time_frac =
      math::clamp((t + goal_time_secs - path_start_time) / path_duration, 0, 1);
  int idx = (int)(time_frac*N);
  if (idx >= N) {
    idx = N - 1;
  }
  PathPointT ref_data = path_data[idx];
  //Convert ref_data to state and controls
  Eigen::VectorXd state(12);
  state.segment(0,3) = std::get<0>(ref_data);//Position
  state.segment(3,3) = std::get<1>(ref_data);//Roll, Pitch, Yaw
  state.segment(6,3) = std::get<2>(ref_data);//Velocity
  state.segment(9,3) = std::get<3>(ref_data);//RPY-rates
  Eigen::VectorXd control(4);
  Eigen::Vector3d acceleration = std::get<4>(ref_data);
  acceleration[2] = acceleration[2] + 9.81;
  control(0) = acceleration.norm() / 9.81;
  control.segment(1,3) = std::get<3>(ref_data); 
  return std::make_pair(state, control);
}

