#pragma once
#include "aerial_autonomy/common/conversions.h"
#include "aerial_autonomy/common/file_utils.h"
#include "aerial_autonomy/common/math.h"
#include "aerial_autonomy/types/particle_state.h"
#include "aerial_autonomy/types/reference_trajectory.h"
#include "aerial_autonomy/types/snap.h"

#include <Eigen/Dense>

/**
* @brief A minimum snap trajectory containing controls, states and timestamps
* Gets state/control information using unconstrained QP
*/
class MinimumSnapReferenceTrajectory
    : public ReferenceTrajectory<ParticleState, Snap> {
public:
  /**
  * @brief Constructor
  * @param der_order_in Order of the derivative subject to optimization
  * @param tau_vec_in Array of time intervals
  * @param path_in nby3 matrix containing waypoints (x,y,z)
  */
  MinimumSnapReferenceTrajectory(const int der_order_in,
                                 const Eigen::VectorXd &tau_vec_in,
                                 const Eigen::MatrixXd &path_in)
      : der_order_(der_order_in), tau_vec_(tau_vec_in), path_(path_in) {
    poly_degree_ = 2 * der_order_ + 1;
    n_segments_ = tau_vec_.size();
    total_dimensions_ = (poly_degree_ + 1) * n_segments_;
    n_unknowns_ = 4 * (n_segments_ - 1);
    n_knowns_ = total_dimensions_ - n_unknowns_;
    if (tau_vec_.size() == 1) {
      equal_A_ = equalA(tau_vec_(0));
      cost_Q_ = costQ(tau_vec_(0));
      poly_coeffs_ = equal_A_.lu().solve(bFixed());
      ts_ = {0, tau_vec_(0)};
    } else {
      idx_ = permutIdx();
      equal_A_ = augA();
      cost_Q_ = augQ();
      b_optimized_ = bOptimized();
      poly_coeffs_ = equal_A_.lu().solve(permutRow(b_optimized_, true));
      ts_ = conversions::vectorEigenToStd(math::cumsumEigen(tau_vec_));
    }
  }

  /**
  * @brief Get Polynomial coefficients
  *
  * @return polynomial coefficients
  */
  Eigen::MatrixXd getP() const { return poly_coeffs_; }

  /**
  * @brief Gets the trajectory information at the specified time using minimum
  * snap
  * @param t Time
  * @return Trajectory state and control
  */
  std::pair<ParticleState, Snap> atTime(double t) const {
    if (ts_.empty() || t < ts_.front() || t > ts_.back()) {
      throw std::out_of_range("Accessed reference trajectory out of bounds");
    }
    auto closest_t = std::lower_bound(ts_.begin(), ts_.end(), t);
    int i = closest_t - ts_.begin();
    double t_tau = t - ts_[i - 1];
    Eigen::MatrixXd states_eigen;
    Eigen::MatrixXd equal_A = equalA(t_tau).bottomRows(5);
    if (i == 0) {
      states_eigen = equal_A * poly_coeffs_.middleRows(0, 10);
    } else {
      states_eigen = equal_A * poly_coeffs_.middleRows(10 * (i - 1), 10);
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
  // clang-format off
  const int der_order_;           ///< Order of derivative (snap = 4)
  const Eigen::VectorXd tau_vec_; ///< Vector of time intervals
  const Eigen::MatrixXd path_;    ///< N by R matrix, N: # of waypoints,
                                  ///< R: Dimension of coordinate sys (ex. x,y,z=3)

  int poly_degree_;      ///< Order of the polynomial (ex. for minimum snap, 9th poly)
  int n_segments_;       ///< # of segments, N-1
  int total_dimensions_; ///< Dimension, (poly_degree_ + 1) * n_segments_
  int n_unknowns_;       ///< # of unknowns (will be optimized), 4 * (n_segments_ - 1)
  int n_knowns_;         ///< # of knowns, total_dimensions_ - n_unknowns_

  Eigen::MatrixXd equal_A_;     ///< Mapping from poly coeffs to endpoint derivatives
  Eigen::MatrixXd cost_Q_;      ///< Cost matrix
  Eigen::MatrixXd b_optimized_; ///< Optimized endpoint derivatives
  Eigen::MatrixXd poly_coeffs_; ///< Polynomial coefficents
  std::vector<double> ts_;      ///< Time stamps corresponding to states
  Eigen::VectorXi idx_;         ///< Indexing for permutation
  // clang-format on

  /**
  * @brief Equality matrix, equal_A_ is a mapping matrix from the coefficents of
  * the
  * polynomial [p0 p1 p2 ... p9] to the endpoint derivatives, [b0, bt]
  * b0: Initial endpoint of the polynomial; [pos, vel, accel, jerk, snap]
  * bt: Final endpoint of the polynomial
  * Ap = [b0 bt] = b
  * @param tau Time interval
  * @param der_order_
  * @return Equality matrix
  */
  Eigen::MatrixXd equalA(double tau) const {
    Eigen::MatrixXd A0, Atau1, Atau;
    A0.setZero(der_order_ + 1, poly_degree_ + 1);
    Atau.setZero(der_order_ + 1, poly_degree_ + 1);
    Eigen::MatrixXd A;
    A.setZero(poly_degree_ + 1, poly_degree_ + 1);
    std::vector<double> tau_exp = math::cumulativeExp(tau, poly_degree_ + 1);
    A0(0, 0) = 1;
    for (int i = 0; i <= der_order_; i++) {
      for (int j = 0; j <= poly_degree_; j++) {
        if (i == 0) {
          if (j >= i) {
            Atau(i, j) = tau_exp[j - i];
          }
        } else {
          Eigen::VectorXd diff_const =
              Eigen::VectorXd::LinSpaced(i, j, j - (i - 1));
          if (j == i) {
            A0(i, j) = diff_const.prod();
          }
          if (j >= i) {
            Atau(i, j) = diff_const.prod() * tau_exp[j - i];
          }
        }
      }
    }
    A.block(0, 0, der_order_ + 1, poly_degree_ + 1) = A0;
    A.block(der_order_ + 1, 0, der_order_ + 1, poly_degree_ + 1) = Atau;
    return A;
  }

  /**
  * @brief Augmented equality matrix, diag(A1, A2, A3, ...)
  * @param tau_vec_
  * @param der_order_
  * @return Augmented equality matrix
  */
  Eigen::MatrixXd augA() {
    Eigen::MatrixXd A;
    A.setZero(total_dimensions_, total_dimensions_);
    // Equality constraints
    for (int i = 0; i < n_segments_; i++) {
      A.block((poly_degree_ + 1) * i, (poly_degree_ + 1) * i,
              (poly_degree_ + 1), (poly_degree_ + 1)) = equalA(tau_vec_(i));
    }
    // Continuity constraints
    // Enforce continuity (velocity, accel, jerk, snap) of
    // the two adjacent polynomials
    for (int i = 0; i < n_segments_ - 1; i++) {
      A.block((poly_degree_ + 1) * i + 6, (poly_degree_ + 1) * (i + 1), 4,
              (poly_degree_ + 1)) =
          -A.block((poly_degree_ + 1) * (i + 1) + 1,
                   (poly_degree_ + 1) * (i + 1), 4, (poly_degree_ + 1));
    }
    return A;
  }

  /**
  * @brief Cost matrix
  * @param tau_vec_
  * @param der_order_
  * @return Cost matrix
  */
  Eigen::MatrixXd costQ(double tau) {
    Eigen::MatrixXd Q;
    Q.setZero(poly_degree_ + 1, poly_degree_ + 1);
    std::vector<double> tau_exp = math::cumulativeExp(tau, poly_degree_ + 3);
    for (int i = 0; i < Q.rows(); i++) {
      for (int j = 0; j < Q.cols(); j++) {
        if (i >= der_order_ && j >= der_order_) {
          Eigen::VectorXd i_m =
              Eigen::VectorXd::LinSpaced(der_order_, i, i - (der_order_ - 1));
          Eigen::VectorXd j_m =
              Eigen::VectorXd::LinSpaced(der_order_, j, j - (der_order_ - 1));
          Eigen::VectorXd ij_m = i_m.cwiseProduct(j_m);
          int idx = (i + j) - (2 * der_order_) + 1;
          Q(i, j) = 2 * ij_m.prod() * tau_exp[idx] / static_cast<double>(idx);
        }
      }
    }
    return Q;
  }

  /**
  * @brief Augmented cost matrix
  * @param tau_vec_
  * @param der_order_
  * @return Augmented cost matrix
  */
  Eigen::MatrixXd augQ() {
    Eigen::MatrixXd Q;
    Q.setZero(total_dimensions_, total_dimensions_);
    for (int i = 0; i < n_segments_; i++) {
      Q.block((poly_degree_ + 1) * i, (poly_degree_ + 1) * i,
              (poly_degree_ + 1), (poly_degree_ + 1)) = costQ(tau_vec_(i));
    }
    return Q;
  }

  /**
  * @brief Index for perumtation
  * @return Eigen vector(int)
  */
  Eigen::VectorXi permutIdx() {
    Eigen::VectorXi idx_app;
    idx_app.setZero(n_unknowns_);
    for (int i = 1; i <= n_segments_ - 1; i++) {
      Eigen::VectorXi idx_tmp = Eigen::VectorXi::LinSpaced(
          4, (poly_degree_ + 1) * i + 1, (poly_degree_ + 1) * i + 4);
      idx_app.segment(4 * (i - 1), 4) = idx_tmp;
    }
    Eigen::VectorXi idx;
    idx.setZero(total_dimensions_);
    int row_idx = 0;
    for (int i = 0; i < total_dimensions_; i++) {
      if (!(idx_app.array() == i).any()) {
        idx(row_idx) = i;
        row_idx++;
      }
    }
    idx.bottomRows(n_unknowns_) = idx_app;
    return idx;
  }

  /**
  * @brief Permute row of a matrix
  * @param mat_eigen Eigen matrix
  * @param is_reverse Direction of perumtation
  * @return result Permuted matrix
  */
  Eigen::MatrixXd permutRow(const Eigen::MatrixXd &mat_eigen, bool is_reverse) {
    Eigen::MatrixXd result;
    result.setZero(total_dimensions_, mat_eigen.cols());
    for (int i = 0; i < total_dimensions_; i++) {
      if (is_reverse) {
        result.row(idx_(i)) = mat_eigen.row(i);
      } else if (!is_reverse) {
        result.row(i) = mat_eigen.row(idx_(i));
      }
    }
    return result;
  }

  /**
  * @brief Known (fixed) endpoint derivatives
  * @param tau_vec_
  * @param der_order_
  * @param path_
  * @return Known endpoint derivatives
  */
  Eigen::MatrixXd bFixed() {
    Eigen::MatrixXd b;
    b.setZero(total_dimensions_, path_.cols());
    b.topRows(1) = path_.topRows(1);
    b.row((der_order_ + 1) + (poly_degree_ + 1) * (n_segments_ - 1)) =
        path_.bottomRows(1);
    for (int i = 0; i < path_.rows() - 2; i++) {
      b.row((der_order_ + 1) + (poly_degree_ + 1) * i) = path_.row(i + 1);
      b.row((2 * der_order_ + 2) + (poly_degree_ + 1) * i) = path_.row(i + 1);
    }
    return b;
  }

  /**
  * @brief Optimized endpoint derivatives
  * @param equal_A_ Augmented equality matrix
  * @param permut_C_ Permutation matrix
  * @param cost_Q_ Augmented cost matrix
  * @param b Known endpoint derivatives
  * @return Optimized endpoint derivatives
  */
  Eigen::MatrixXd bOptimized() {
    Eigen::MatrixXd b_Opt = permutRow(bFixed(), false);
    Eigen::MatrixXd bF = b_Opt.topRows(total_dimensions_ - n_unknowns_);
    Eigen::MatrixXd R = permutRow(
        permutRow(equal_A_.transpose().lu().solve(
                      equal_A_.transpose().lu().solve(cost_Q_).transpose()),
                  false)
            .transpose(),
        false);
    Eigen::MatrixXd Rfp = R.topRightCorner(n_knowns_, n_unknowns_);
    Eigen::MatrixXd Rpp = R.bottomRightCorner(n_unknowns_, n_unknowns_);
    Eigen::MatrixXd bP_Opt = -Rpp.lu().solve(Rfp.transpose()) * bF;
    b_Opt.bottomRows(n_unknowns_) = bP_Opt;
    return b_Opt;
  }
};
