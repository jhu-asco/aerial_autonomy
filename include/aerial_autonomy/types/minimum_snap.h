#pragma once
#include "aerial_autonomy/types/reference_trajectory.h"

#include <utility>
#include <Eigen/Dense>
#include <vector>

using namespace Eigen;

class MinimumSnapReferenceTrajectory: public ReferenceTrajectory<ParticleState, Snap>
{
public:
  /*Constructor*/
  Cow(const int r_in, const Ref<const VectorXd>& tau_vec_in, const Ref<const MatrixXd>& path_in) // normal non-default constructor
  : r(r_in), tau_vec(tau_vec_in), path(path_in)
  {
    Init(); // Separate function
  }

  //void Init(int r_in, const Ref<const VectorXd>& tau_vec_in, const Ref<const MatrixXd>& path_in);
  // MinimumSnapReferenceTrajectory Member functions
  void Init()
  {
    n = 2*r + 1; // order of the polynomial (ex. for minimum snap, 9th poly)
    m = tau_vec.size(); // Number of segments
    A = augA();
    C = permutC();
    Q = augQ();
    bOpt = bOptimized();
    P = A.lu().solve(C.transpose())*bOpt;
    ts_eigen = cumsumEigen();
  }

  MatrixXd getP() const { return P; }
  VectorXd getTs() const { return ts_eigen; }
  MatrixXd getEqualA(double t_tau) const { return equalA(t_tau); }

  std::pair<ParticleState, Snap> atTime(double t) const
  {
    VectorXd ts_eigen = getTs();
    // conversion
    //std::vector<double> ts_std(ts_eigen.data(), ts_eigen.data() + ts_eigen.rows() * ts_eigen.cols());
    std::vector<double> ts_std(ts_eigen.data(), ts_eigen.data() + ts_eigen.rows() * ts_eigen.cols());
    // find interator
    auto closest_t = std::lower_bound(ts_std.begin(), ts_std.end(), t);
    int i = closest_t - ts_std.begin();

    //
    double t_tau = t - ts_std[i-1];
    MatrixXd P = getP();
    // Select the segment of polynomials
    MatrixXd P_seg = P.middleRows(10*(i-1), 10);

    // borrow equalA to get pos, vel, ...
    MatrixXd states_eigen = equalA(t_tau).bottomRows(5)*P_seg;

    // Type conversion
    Position p(states_eigen(0,0), states_eigen(0,1), states_eigen(0,2));
    Velocity v(states_eigen(1,0), states_eigen(1,1), states_eigen(1,2));
    Acceleration a(states_eigen(2,0), states_eigen(2,1), states_eigen(2,2));
    Jerk j(states_eigen(3,0), states_eigen(3,1), states_eigen(3,2));
    Snap snap(states_eigen(4,0), states_eigen(4,1), states_eigen(4,2));

    return std::pair<ParticleState, Snap> (ParticleState(p, v, a, j), snap);
  }

private:
  // member variables
  const int r;
  const VectorXd tau_vec;
  const MatrixXd path;
  int n;
  int m;
  MatrixXd A;
  MatrixXd C;
  MatrixXd Q;
  //VectorXd bfixed;
  MatrixXd bOpt;
  MatrixXd P;
  VectorXd ts_eigen;

  /**
  * @brief Time stamps corresponding to states
  */
  std::vector<double> ts_std;

  // member functions

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
  //MatrixXd equalA(double tau) const;
  MatrixXd equalA(double tau) const
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
    MatrixXd A; A.setZero((n+1)*m, (n+1)*m);
    for (int i = 0; i < m ; i++)
    {
      A.block((n+1)*i, (n+1)*i, (n+1), (n+1)) = equalA(tau_vec(i));
    }
    /* Continuity constraint */
    // Enforce continuity (velocity, accel, jerk, snap) of the two adjacent polynomials
    for (int i = 0; i < m-1; i++)
    {
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
    MatrixXd Q; Q.setZero(n+1,n+1);
    for (int i = 0; i < Q.rows(); i++)
    {
      for (int j = 0; j < Q.cols(); j++)
      {
        if (i >= r && j >= r)
        {
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
  MatrixXd augQ()
  {
    MatrixXd Q; Q.setZero((n+1)*m, (n+1)*m);
    for (int i = 0; i < m ; i++)
    {
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
    for (int i = 1; i <= m-1; i++)
    {
      VectorXd idx_tmp = VectorXd::LinSpaced(4, (n+1)*i+1, (n+1)*i+4);
      idx.segment(4*(i-1), 4) = idx_tmp;
    }
    MatrixXd C; C.setZero((n+1)*m, (n+1)*m);
    MatrixXd C_app; C_app.setZero(4*(m-1), (n+1)*m);
    int C_row_idx = 0;
    int C_app_row_idx = 0;
    for (int i = 0; i < (n+1)*m; i++)
    {
      if ((idx.array() == i).any())
      {
        C_app(C_app_row_idx, i) = 1;
        C_app_row_idx++;
      } else
      {
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
  MatrixXd bFixed()
  {
    int idx_free = 4*(m-1); // # of unknowns (subject to optimization)
    MatrixXd b; b.setZero((n+1)*m, path.cols());
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
  MatrixXd bOptimized()
  {
    //int m = bfixed.rows()/(n+1);
    int m = bFixed().rows()/(n+1);
    //VectorXd b_Opt = C*bfixed;
    MatrixXd b_Opt = C*bFixed();
    MatrixXd bF = b_Opt.topRows((n+1)*m - 4*(m-1));
    MatrixXd R = C*A.transpose().lu().solve(Q)*A.lu().solve(C.transpose());
    MatrixXd Rfp = R.topRightCorner((n+1)*m - 4*(m-1), 4*(m-1));
    MatrixXd Rpp = R.bottomRightCorner(4*(m-1), 4*(m-1));
    MatrixXd bP_Opt = -Rpp.lu().solve(Rfp.transpose())*bF;
    b_Opt.bottomRows(4*(m-1)) = bP_Opt;
    return b_Opt;
  }

  VectorXd cumsumEigen()
  {
    //VectorXd ts_eigen;
    ts_eigen.setZero(m+1);
    for (int i = 1; i < m+1; i++) {
      /* code */
      //std::cout << "/* message */" << tau_vec.head(i) << '\n';
      ts_eigen(i) = tau_vec.head(i).sum();
    }
    return ts_eigen;
  }

};
