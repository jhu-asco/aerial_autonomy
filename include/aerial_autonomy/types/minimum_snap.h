#pragma once
#include <utility>
#include <Eigen/Dense>
#include "aerial_autonomy/types/reference_trajectory.h"
#include <vector>

using namespace Eigen;

class MinimumSnapReferenceTrajectory
{
public:
  void setValues(int r_in, VectorXd tau_vec_in, MatrixXd path_in) // public, can be accessed by anyone
  {
    r = r_in;
    tau_vec = tau_vec_in;
    path = path_in;
    n = 2*r_in + 1; // order of the polynomial (ex. for minimum snap, 9th poly)
    m = tau_vec.size(); // Number of segments
    A = augA();
    C = permutC();
    Q = augQ();
    bfixed = bFixed();
    bOpt = bOptimized(A, C, Q, bfixed);
    P = A.lu().solve(C.transpose())*bOpt;
  }

  void printP()
  {
    std::cout << "r: " << r << " tau_vec: " << tau_vec << " path: " << path << '\n'
    << "n: " << n << " m: " << m << '\n' << "A: " << A << '\n';
    std::cout << "Optimized coefficents: " << '\n' << P << '\n';
  }

private:
  int r;  // private by default, can only be accessed by other members
  VectorXd tau_vec;   // private by default, can only be accessed by other members
  MatrixXd path;   // private by default, can only be accessed by other members
  int n;
  int m;
  MatrixXd A;
  MatrixXd C;
  MatrixXd Q;
  VectorXd bfixed;
  VectorXd bOpt;
  VectorXd P;

  /* Equality matrix, A */
  // Equality matrix, A is a mapping matrix from the coefficents of the polynomial
  // , [p0 p1 p2 ... p9] to the equality constraint, [b0, bt]
  // b0: start point of the polynomial; [pos, vel, accel, jerk, snap]
  // bt: end point of the polynomial
  // Ap = [b0 bt] = b

  // Input:
  // tau = time interval
  // r = The order of the differentiation subject to optimization (ex. for
  // minimum snap, r = 4)
  // Output:
  // A = equality matrix
  MatrixXd equalA(double tau)
  {
    MatrixXd A0, Atau1, Atau; A0.setZero(r+1,n+1); Atau.setZero(r+1,n+1);
    MatrixXd A; A.setZero(n+1,n+1);
    A0(0,0) = 1;
    for (int i = 0; i <= r; i++) {
      for (int j = 0; j <= n; j++) {
        if (i == 0) {
          if (j >= i) {
            Atau(i,j) = pow(tau, j-i);
          }
        } else {
          VectorXd m = VectorXd::LinSpaced(i, j, j-(i-1));
          if (j == i) {
            A0(i,j) = m.prod();
          }
          if (j >= i) {
            Atau(i,j) = m.prod()*pow(tau, j-i);
          }
        }
      }
    }
    A.block(0,0,r+1,n+1) = A0;
    A.block(r+1,0,r+1,n+1) = Atau;
    return A;
  }

  /* Augmented equality matrices, A1, A2, A3, ... */
  // Input:
  // tau_vec = array of time intervals
  // r = The order of the differentiation subject to optimization (ex. for
  // minimum snap, r = 4)
  // Output:
  // A = augmented equality matrix
  MatrixXd augA()
  {
    //int n = 2*r + 1;
    //int m = tau_vec.size(); // Number of segments
    MatrixXd A; A.setZero((n+1)*m, (n+1)*m);
    for (int i = 0; i < m ; i++) {
     A.block((n+1)*i, (n+1)*i, (n+1), (n+1)) = equalA(tau_vec(i));
     //cout << "equalA is " << '\n' << equalA(tau_vec(i), r) << '\n';
    }
    /* Continuity constraint */
    // Enforce continuity (velocity, accel, jerk, snap) of the two adjacent polynomials
    for (int i = 0; i < m-1; i++) {
      A.block((n+1)*i+6, (n+1)*(i+1), 4, (n+1)) = -A.block((n+1)*(i+1)+1, (n+1)*(i+1), 4, (n+1));
    }
    return A;
  }

  /* Cost matrix Q */
  // Input:
  // tau_vec = time interval
  // r = The order of the differentiation subject to optimization (ex. for
  // minimum snap, r = 4)
  // Output:
  // Q = cost matrix
  MatrixXd costQ(double tau)
  {
    //int n = 2*r +1;
    MatrixXd Q; Q.setZero(n+1,n+1);
    for (int i = 0; i < Q.rows(); i++) {
      for (int j = 0; j < Q.cols(); j++) {
        if (i >= r && j >= r) {
          VectorXd i_m = VectorXd::LinSpaced(r, i, i-(r-1));
          VectorXd j_m = VectorXd::LinSpaced(r, j, j-(r-1));
          VectorXd ij_m = i_m.cwiseProduct(j_m);
          Q(i,j) = 2*ij_m.prod()*pow(tau, (i+j-2*r+1))/(i+j-2*r+1);
        }
      }
    }
    return Q;
  }

  /* Augment cost matrix, Q*/
  // Input:
  // tau_vec = array of time intervals
  // r = The order of the differentiation subject to optimization (ex. for
  // minimum snap, r = 4)
  // Output:
  // Q = augmented cost matrix
  MatrixXd augQ() {
    //int n = 2*r + 1;
    //int m = tau_vec.size();
    MatrixXd Q; Q.setZero((n+1)*m, (n+1)*m);
    for (int i = 0; i < m ; i++) {
     Q.block((n+1)*i, (n+1)*i, (n+1), (n+1)) = costQ(tau_vec(i));
    }
    return Q;
  }

  /* Permutation matrix, C */
  // Input:
  // tau_vec = array of time intervals
  // r = The order of the differentiation subject to optimization (ex. for
  // minimum snap, r = 4)
  // Output:
  // C = permutation matrix
  MatrixXd permutC()
  {
    int n = 2*r + 1;
    int m = tau_vec.size();
    VectorXd idx; idx.setZero(4*(m-1));
    for (int i = 1; i <= m-1; i++) {
      VectorXd idx_tmp = VectorXd::LinSpaced(4, (n+1)*i+1, (n+1)*i+4);
      idx.segment(4*(i-1), 4) = idx_tmp;
    }
    MatrixXd C; C.setZero((n+1)*m, (n+1)*m);
    MatrixXd C_app; C_app.setZero(4*(m-1), (n+1)*m);
    int C_row_idx = 0;
    int C_app_row_idx = 0;
    for (int i = 0; i < (n+1)*m; i++) {
      if ((idx.array() == i).any()) {
        C_app(C_app_row_idx, i) = 1;
        C_app_row_idx++;
      } else {
        C(C_row_idx, i) = 1;
        C_row_idx++;
      }
    }
    C.bottomRows(4*(m-1)) = C_app;
    return C;
  }

  /* Known (fixed) endpoint derivatives, b */
  // Input:
  // path = m by p matrix; m = number of waypoints, p = number of dimension
  // r = The order of the differentiation subject to optimization (ex. for
  // minimum snap, r = 4)
  // Output:
  // b = known endpoint derivatives
  VectorXd bFixed()
  {
    int idx_free = 4*(m-1); // # of unknowns (subject to optimization)
    VectorXd b; b.setZero((n+1)*m, path.cols());
    b.topRows(1) = path.topRows(1);
    b.row((r+1)+(n+1)*(m-1)) = path.bottomRows(1);
    for (int i = 0; i < path.rows()-2; i++) {
      b.row((r+1)+(n+1)*i) = path.row(i+1);
      b.row((2*r+2)+(n+1)*i) = path.row(i+1);
    }
    return b;
  }

  /* Optimized endpoint derivatives */
  // Input:
  // MatrixXd A, MatrixXd C, MatrixXd Q, VectorXd b
  // r = The order of the differentiation subject to optimization (ex. for
  // minimum snap, r = 4)
  // Output:
  // b_Opt = optimized endpoint derivatives
  VectorXd bOptimized(MatrixXd A, MatrixXd C, MatrixXd Q, VectorXd b)
  {
    int m = b.rows()/(n+1);
    VectorXd b_Opt = C*b;
    VectorXd bF = b_Opt.topRows((n+1)*m - 4*(m-1));
    MatrixXd R = C*A.transpose().lu().solve(Q)*A.lu().solve(C.transpose());
    MatrixXd Rfp = R.topRightCorner((n+1)*m - 4*(m-1), 4*(m-1));
    MatrixXd Rpp = R.bottomRightCorner(4*(m-1), 4*(m-1));
    VectorXd bP_Opt = -Rpp.lu().solve(Rfp.transpose())*bF;
    b_Opt.bottomRows(4*(m-1)) = bP_Opt;
    return b_Opt;
  }
  //VectorXd b_Opt = bOptimized(A, C, Q, b);
  //cout << "b_Opt is " << '\n' << b_Opt << '\n' << "size of b_Opt is " << b_Opt.rows() << "by" << b_Opt.cols() << '\n';
  //VectorXd P = A.lu().solve(C.transpose())*b_Opt;
};
