#include "aerial_autonomy/common/math.h"
#include "aerial_autonomy/common/conversions.h"

#include <Eigen/Dense>
#include <algorithm>
#include <armadillo>

/**
* @brief Math namespace to separate math functions from
* system functions if exist
*/
namespace math {

double angleWrap(double x) {
  x = std::fmod(x + M_PI, 2 * M_PI);
  if (x < 0)
    x += 2 * M_PI;
  return x - M_PI;
}

double clamp(double x, double min, double max) {
  return std::min(std::max(x, min), max);
}

double map(double input, double input_min, double input_max, double output_min,
           double output_max) {
  if (input > input_max)
    return output_max;

  else if (input < input_min)
    return output_min;

  return (output_min +
          ((input - input_min) * (output_max - output_min)) /
              (input_max - input_min));
}

Eigen::Matrix3d hat(const Eigen::Vector3d &v) {
  Eigen::Matrix3d v_hat;
  v_hat << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return v_hat;
}

Eigen::Quaterniond rpyToQuat(double r, double p, double y) {
  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX());
  return q;
}

Eigen::MatrixXd sylvester(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                          const Eigen::MatrixXd &C) {
  arma::mat A_arma = conversions::eigenToArma(A);
  arma::mat B_arma = conversions::eigenToArma(B);
  arma::mat C_arma = conversions::eigenToArma(C);

  arma::mat X_arma = arma::syl(A_arma, B_arma, C_arma);

  return conversions::armaToEigen(X_arma);
}

Eigen::VectorXd cumsumEigen(const Eigen::VectorXd &vec_eigen) {
  Eigen::VectorXd vec_cumsum_eigen;
  int vec_size = vec_eigen.size();
  vec_cumsum_eigen.setZero(vec_size + 1);
  for (int i = 0; i < vec_size; i++) {
    vec_cumsum_eigen(i + 1) = vec_cumsum_eigen(i) + vec_eigen(i);
  }
  return vec_cumsum_eigen;
}

std::vector<double> cumulativeExp(double tau, int size) {
  std::vector<double> result(size);
  result[0] = 1;
  for (int i = 1; i < size; i++) {
    result[i] = result[i - 1] * tau;
  }
  return result;
}
}
