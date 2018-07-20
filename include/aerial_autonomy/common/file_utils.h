#pragma once
#include <Eigen/Dense>
#include <fstream>
#include <vector>

/**
* @brief Read csv file as a type Eigen::MatrixXd
* @param path Complete path to the csv directory
* @return A matrix with type Eigen::MatrixXd
*/
namespace file_utils {

template <typename M> M load_csv(const std::string &path) {
  std::ifstream indata;
  indata.open(path);
  std::string line;
  std::vector<double> values;
  uint rows = 0;
  while (std::getline(indata, line)) {
    std::stringstream lineStream(line);
    std::string cell;
    while (std::getline(lineStream, cell, ',')) {
      values.push_back(std::stod(cell));
    }
    ++rows;
  }
  return Eigen::Map<
      const Eigen::Matrix<typename M::Scalar, M::RowsAtCompileTime,
                          M::ColsAtCompileTime, Eigen::RowMajor>>(
      values.data(), rows, values.size() / rows);
}
}
