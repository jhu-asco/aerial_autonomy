#pragma once
#include "aerial_autonomy/types/particle_state.h"
#include "aerial_autonomy/types/reference_trajectory.h"
#include "aerial_autonomy/types/snap.h"

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <utility>
#include <vector>

using namespace Eigen;
/**
* @brief A minimum snap trajectory containing controls, states and timestamps
* Gets state/control information using unconstrained QP
*/
class MinimumSnapReferenceTrajectory
    : public ReferenceTrajectory<ParticleState, Snap> {
public:
  /**
  * @brief Constructor for more than two segments
  * @param r_in Order of the derivative subject to optimization
  * @param tau_vec_in Array of time intervals
  * @param path_in nby3 matrix containing waypoints (x,y,z)
  */
  MinimumSnapReferenceTrajectory(const int r_in,
                                 const Ref<const VectorXd> &tau_vec_in,
                                 const Ref<const MatrixXd> &path_in)
      : r(r_in), tau_vec(tau_vec_in), path(path_in) {
    Init();
  }
  /**
  * @brief Initialization for more than two segments
  */
  void Init() {
    n = 2 * r + 1;
    m = tau_vec.size();
    A = augA();
    C = permutC();
    Q = augQ();
    bOpt = bOptimized();
    P = A.lu().solve(C.transpose()) * bOpt;
    ts = vecEigenToStd(cumsumEigen(tau_vec));
  }
  /**
  * @brief Constructor for a single segment
  * @param r_in Order of the derivative subject to optimization
  * @param tau Time interval
  * @param path_in 1by3 matrix containing waypoints (x,y,z)
  */
  MinimumSnapReferenceTrajectory(const int r_in, const double &tau,
                                 const Ref<const MatrixXd> &path_in)
      : r(r_in), path(path_in) {
    InitSingleSeg(tau);
  }
  /**
  * @brief Initialization for a single segment
  */
  void InitSingleSeg(double tau) {
    n = 2 * r + 1;
    m = 1;
    A = equalA(tau);
    Q = costQ(tau);
    P = A.lu().solve(bFixed());
    ts = {0, tau};
  }
  /**
  * @brief Convertor from Eigen::VectorXd to std::vector<double>
  * @param vec_eigen Eigen::VectorXd
  * @return std::vector<double>
  */
  std::vector<double> vecEigenToStd(const Ref<const VectorXd> &vec_eigen) {
    std::vector<double> vec_std(vec_eigen.data(),
                                vec_eigen.data() +
                                    vec_eigen.rows() * vec_eigen.cols());
    return vec_std;
  }

  MatrixXd getP() const { return P; }

  MatrixXd getEqualA(double t_tau) const { return equalA(t_tau); }

  /**
  * @brief Gets the trajectory information at the specified time using minimum
  * snap
  * @param t Time
  * @return Trajectory state and control
  */
  std::pair<ParticleState, Snap> atTime(double t) const {
    if (ts.empty() || t < ts.front() || t > ts.back()) {
      throw std::out_of_range("Accessed reference trajectory out of bounds");
    }
    auto closest_t = std::lower_bound(ts.begin(), ts.end(), t);
    int i = closest_t - ts.begin();
    double t_tau = t - ts[i - 1];
    MatrixXd states_eigen;
    if (i == 0) {
      states_eigen = equalA(t_tau).bottomRows(5) * P.middleRows(0, 10);
    } else {
      states_eigen =
          equalA(t_tau).bottomRows(5) * P.middleRows(10 * (i - 1), 10);
    }
    // Type conversion
    Position p(states_eigen(0, 0), states_eigen(0, 1), states_eigen(0, 2));
    Velocity v(states_eigen(1, 0), states_eigen(1, 1), states_eigen(1, 2));
    Acceleration a(states_eigen(2, 0), states_eigen(2, 1), states_eigen(2, 2));
    Jerk j(states_eigen(3, 0), states_eigen(3, 1), states_eigen(3, 2));
    Snap snap(states_eigen(4, 0), states_eigen(4, 1), states_eigen(4, 2));

    return std::pair<ParticleState, Snap>(ParticleState(p, v, a, j), snap);
  }

private:
  const int r;
  const VectorXd tau_vec;
  const MatrixXd path;
  int n;      ///< order of the polynomial (ex. for minimum snap, 9th poly)
  int m;      ///< Number of segments
  MatrixXd A; ///< Mapping from polynomial coefficents to endpoint derivatives
  MatrixXd C; ///< Permutation matrix
  MatrixXd Q; ///< Cost matrix
  MatrixXd bOpt;          ///< Optimized endpoint derivatives
  MatrixXd P;             ///< Polynomial coefficents
  std::vector<double> ts; ///< Time stamps corresponding to states
                          /**
                          * @brief Equality matrix, A is a mapping matrix from the coefficents of the
                          * polynomial [p0 p1 p2 ... p9] to the endpoint derivatives, [b0, bt]
                          * b0: Initial endpoint of the polynomial; [pos, vel, accel, jerk, snap]
                          * bt: Final endpoint of the polynomial
                          * Ap = [b0 bt] = b
                          * @param tau Time interval
                          * @param r
                          * @return Equality matrix
                          */
  MatrixXd equalA(double tau) const {
    MatrixXd A0, Atau1, Atau;
    A0.setZero(r + 1, n + 1);
    Atau.setZero(r + 1, n + 1);
    MatrixXd A;
    A.setZero(n + 1, n + 1);
    A0(0, 0) = 1;
    for (int i = 0; i <= r; i++) {
      for (int j = 0; j <= n; j++) {
        if (i == 0) {
          if (j >= i) {
            Atau(i, j) = pow(tau, j - i);
          }
        } else {
          VectorXd m = VectorXd::LinSpaced(i, j, j - (i - 1));
          if (j == i) {
            A0(i, j) = m.prod();
          }
          if (j >= i) {
            Atau(i, j) = m.prod() * pow(tau, j - i);
          }
        }
      }
    }
    A.block(0, 0, r + 1, n + 1) = A0;
    A.block(r + 1, 0, r + 1, n + 1) = Atau;
    return A;
  }
  /**
  * @brief Augmented equality matrix, diag(A1, A2, A3, ...)
  * @param tau_vec
  * @param r
  * @return Augmented equality matrix
  */
  MatrixXd augA() {
    MatrixXd A;
    A.setZero((n + 1) * m, (n + 1) * m);
    // Equality constraints
    for (int i = 0; i < m; i++) {
      A.block((n + 1) * i, (n + 1) * i, (n + 1), (n + 1)) = equalA(tau_vec(i));
    }
    // Continuity constraints
    // Enforce continuity (velocity, accel, jerk, snap) of
    // the two adjacent polynomials
    for (int i = 0; i < m - 1; i++) {
      A.block((n + 1) * i + 6, (n + 1) * (i + 1), 4, (n + 1)) =
          -A.block((n + 1) * (i + 1) + 1, (n + 1) * (i + 1), 4, (n + 1));
    }
    return A;
  }
  /**
  * @brief Cost matrix
  * @param tau_vec
  * @param r
  * @return Cost matrix
  */
  MatrixXd costQ(double tau) {
    MatrixXd Q;
    Q.setZero(n + 1, n + 1);
    for (int i = 0; i < Q.rows(); i++) {
      for (int j = 0; j < Q.cols(); j++) {
        if (i >= r && j >= r) {
          VectorXd i_m = VectorXd::LinSpaced(r, i, i - (r - 1));
          VectorXd j_m = VectorXd::LinSpaced(r, j, j - (r - 1));
          VectorXd ij_m = i_m.cwiseProduct(j_m);
          Q(i, j) = 2 * ij_m.prod() * pow(tau, (i + j - 2 * r + 1)) /
                    (i + j - 2 * r + 1);
        }
      }
    }
    return Q;
  }
  /**
  * @brief Augmented cost matrix
  * @param tau_vec
  * @param r
  * @return Augmented cost matrix
  */
  MatrixXd augQ() {
    MatrixXd Q;
    Q.setZero((n + 1) * m, (n + 1) * m);
    for (int i = 0; i < m; i++) {
      Q.block((n + 1) * i, (n + 1) * i, (n + 1), (n + 1)) = costQ(tau_vec(i));
    }
    return Q;
  }
  /**
  * @brief Permutation matrix
  * @param tau_vec
  * @param r
  * @return Permutation matrix
  */
  MatrixXd permutC() {
    VectorXd idx;
    idx.setZero(4 * (m - 1));
    for (int i = 1; i <= m - 1; i++) {
      VectorXd idx_tmp =
          VectorXd::LinSpaced(4, (n + 1) * i + 1, (n + 1) * i + 4);
      idx.segment(4 * (i - 1), 4) = idx_tmp;
    }
    MatrixXd C;
    C.setZero((n + 1) * m, (n + 1) * m);
    MatrixXd C_app;
    C_app.setZero(4 * (m - 1), (n + 1) * m);
    int C_row_idx = 0;
    int C_app_row_idx = 0;
    for (int i = 0; i < (n + 1) * m; i++) {
      if ((idx.array() == i).any()) {
        C_app(C_app_row_idx, i) = 1;
        C_app_row_idx++;
      } else {
        C(C_row_idx, i) = 1;
        C_row_idx++;
      }
    }
    C.bottomRows(4 * (m - 1)) = C_app;
    return C;
  }
  /**
  * @brief Known (fixed) endpoint derivatives
  * @param tau_vec
  * @param r
  * @param path
  * @return Known endpoint derivatives
  */
  MatrixXd bFixed() {
    // int idx_free = 4*(m-1); // # of unknowns (subject to optimization)
    MatrixXd b;
    b.setZero((n + 1) * m, path.cols());
    b.topRows(1) = path.topRows(1);
    b.row((r + 1) + (n + 1) * (m - 1)) = path.bottomRows(1);
    for (int i = 0; i < path.rows() - 2; i++) {
      b.row((r + 1) + (n + 1) * i) = path.row(i + 1);
      b.row((2 * r + 2) + (n + 1) * i) = path.row(i + 1);
    }
    return b;
  }
  /**
  * @brief Optimized endpoint derivatives
  * @param A Augmented equality matrix
  * @param C Permutation matrix
  * @param Q Augmented cost matrix
  * @param b Known endpoint derivatives
  * @return Optimized endpoint derivatives
  */
  MatrixXd bOptimized() {
    int m = bFixed().rows() / (n + 1);
    MatrixXd b_Opt = C * bFixed();
    MatrixXd bF = b_Opt.topRows((n + 1) * m - 4 * (m - 1));
    MatrixXd R = C * A.transpose().lu().solve(Q) * A.lu().solve(C.transpose());
    MatrixXd Rfp = R.topRightCorner((n + 1) * m - 4 * (m - 1), 4 * (m - 1));
    MatrixXd Rpp = R.bottomRightCorner(4 * (m - 1), 4 * (m - 1));
    MatrixXd bP_Opt = -Rpp.lu().solve(Rfp.transpose()) * bF;
    b_Opt.bottomRows(4 * (m - 1)) = bP_Opt;
    return b_Opt;
  }
  /**
  * @brief Cumulative sum of a vector
  * @param vec_eigen Eigen::VectorXd
  * @return Cumulative sum of a vector
  */
  VectorXd cumsumEigen(const Ref<const VectorXd> &vec_eigen) {
    VectorXd vec_cumsum_eigen;
    vec_cumsum_eigen.setZero(m + 1);
    for (int i = 1; i < m + 1; i++) {
      vec_cumsum_eigen(i) = vec_eigen.head(i).sum();
    }
    return vec_cumsum_eigen;
  }
};

/**
* @brief Read csv file as a type Eigen::MatrixXd
* @param path Complete path to the csv directory
* @return A matrix with type Eigen::MatrixXd
*/
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
  return Map<const Matrix<typename M::Scalar, M::RowsAtCompileTime,
                          M::ColsAtCompileTime, RowMajor>>(
      values.data(), rows, values.size() / rows);
};
