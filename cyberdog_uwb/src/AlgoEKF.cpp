//
// File: AlgoEKF.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 21-Apr-2023 15:42:45
//

// Include Files
#include "cyberdog_uwb/AlgoEKF.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : void
// Return Type  : void
//
AlgoEKF::AlgoEKF()
{
  static const double dv[16]{0.01, 0.0, 0.0,  0.0, 0.0, 0.01, 0.0, 0.0,
                             0.0,  0.0, 0.01, 0.0, 0.0, 0.0,  0.0, 0.01};
  static const double dv1[16]{0.060820812668254558,
                              0.0,
                              0.048712226664389985,
                              0.0,
                              0.0,
                              0.060820812668254558,
                              0.0,
                              0.048712226664389985,
                              0.048712226664389985,
                              0.0,
                              0.24923931000597038,
                              0.0,
                              0.0,
                              0.048712226664389985,
                              0.0,
                              0.24923931000597038};
  static const double dv3[16]{1.0,      0.0, 0.0, 0.0, 0.0, 1.0,      0.0, 0.0,
                              0.047364, 0.0, 1.0, 0.0, 0.0, 0.047364, 0.0, 1.0};
  static const double dv2[8]{
      0.20273604222751521, 0.0, 0.16237408888129995, 0.0, 0.0,
      0.20273604222751521, 0.0, 0.16237408888129995};
  static const signed char iv[8]{1, 0, 0, 1, 0, 0, 0, 0};
  R[0] = 0.3;
  R[1] = 0.0;
  R[2] = 0.0;
  R[3] = 0.3;
  std::copy(&dv[0], &dv[16], &Q[0]);
  std::copy(&dv1[0], &dv1[16], &P[0]);
  for (int i{0}; i < 8; i++) {
    H[i] = iv[i];
    K[i] = dv2[i];
  }
  Z[0] = 5.15;
  Z[1] = 0.177124;
  std::copy(&dv3[0], &dv3[16], &A[0]);
  X[0] = 5.3941433862481825;
  X[1] = 0.17500709524468328;
  X[2] = -0.23133286650291757;
  X[3] = -0.0065269809015327329;
  num_not_updated = 0.0;
  update_flag = 1.0;
}

//
// Arguments    : void
// Return Type  : void
//
AlgoEKF::~AlgoEKF() = default;

//
// EKF_INIT EKF初始化
//
// Arguments    : double dis_init
//                double theta_init
//                double Q_var
//                double R_var
// Return Type  : void
//
void AlgoEKF::EKF_init(double dis_init, double theta_init, double Q_var,
                       double R_var)
{
  static const double dv[16]{0.1, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                             0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.1};
  static const signed char iv[8]{1, 0, 0, 1, 0, 0, 0, 0};
  for (int i{0}; i < 8; i++) {
    H[i] = iv[i];
  }
  std::copy(&dv[0], &dv[16], &P[0]);
  update_flag = 1.0;
  num_not_updated = 0.0;
  X[0] = dis_init;
  X[1] = theta_init;
  X[2] = 0.0;
  X[3] = 0.0;
  //  X = zeros(4,1);
  //  X(1) = dis_init;
  //  X(2) = theta_init;
  std::memset(&A[0], 0, 16U * sizeof(double));
  Z[0] = 0.0;
  Z[1] = 0.0;
  std::memset(&K[0], 0, 8U * sizeof(double));
  Q[0] = Q_var;
  Q[4] = 0.0;
  Q[8] = 0.0;
  Q[12] = 0.0;
  Q[1] = 0.0;
  Q[5] = Q_var;
  Q[9] = 0.0;
  Q[13] = 0.0;
  Q[2] = 0.0;
  Q[6] = 0.0;
  Q[10] = Q_var;
  Q[14] = 0.0;
  Q[3] = 0.0;
  Q[7] = 0.0;
  Q[11] = 0.0;
  Q[15] = Q_var;
  R[0] = R_var;
  R[2] = 0.0;
  R[1] = 0.0;
  R[3] = R_var;
}

//
// EKF_PRIDECT EKF预测过程
//
// Arguments    : double dt
// Return Type  : void
//
void AlgoEKF::EKF_pridect(double dt)
{
  double dv1[16];
  double dv[4];
  double d;
  double d1;
  double d2;
  double d3;
  double dt_idx_0_tmp;
  int P_tmp;
  A[0] = 1.0;
  A[4] = 0.0;
  A[8] = dt;
  A[12] = 0.0;
  A[1] = 0.0;
  A[5] = 1.0;
  A[9] = 0.0;
  A[13] = dt;
  dt_idx_0_tmp = dt * dt / 2.0;
  A[2] = 0.0;
  A[3] = 0.0;
  A[6] = 0.0;
  A[7] = 0.0;
  A[10] = 1.0;
  A[11] = 0.0;
  A[14] = 0.0;
  A[15] = 1.0;
  dt_idx_0_tmp =
      ((dt_idx_0_tmp * 0.004 + dt_idx_0_tmp * 0.001) + dt * 0.0) + dt * 0.0;
  d = X[0];
  d1 = X[1];
  d2 = X[2];
  d3 = X[3];
  for (int i{0}; i < 4; i++) {
    dv[i] = (((A[i] * d + A[i + 4] * d1) + A[i + 8] * d2) + A[i + 12] * d3) +
            dt_idx_0_tmp;
  }
  for (int i{0}; i < 4; i++) {
    X[i] = dv[i];
    d = A[i];
    d1 = A[i + 4];
    d2 = A[i + 8];
    d3 = A[i + 12];
    for (int i1{0}; i1 < 4; i1++) {
      P_tmp = i1 << 2;
      dv1[i + P_tmp] =
          ((d * P[P_tmp] + d1 * P[P_tmp + 1]) + d2 * P[P_tmp + 2]) +
          d3 * P[P_tmp + 3];
    }
  }
  for (int i{0}; i < 4; i++) {
    d = dv1[i];
    d1 = dv1[i + 4];
    d2 = dv1[i + 8];
    d3 = dv1[i + 12];
    for (int i1{0}; i1 < 4; i1++) {
      P_tmp = i + (i1 << 2);
      P[P_tmp] =
          (((d * A[i1] + d1 * A[i1 + 4]) + d2 * A[i1 + 8]) + d3 * A[i1 + 12]) +
          Q[P_tmp];
    }
  }
}

//
// EKF_UPDATE EKF更新过程
//
// Arguments    : double dis
//                double theta
//                double threshold
// Return Type  : void
//
void AlgoEKF::EKF_update(double dis, double theta, double threshold)
{
  double dv2[16];
  double dv[8];
  double y_tmp[8];
  double x[4];
  double e[2];
  double b_tmp_idx_0;
  double b_tmp_idx_1;
  double b_tmp_idx_2;
  double d;
  double d1;
  double d2;
  double d3;
  double r;
  double t;
  int i2;
  int x_tmp;
  Z[0] = dis;
  Z[1] = theta;
  for (int i{0}; i < 2; i++) {
    d = 0.0;
    for (int i1{0}; i1 < 4; i1++) {
      i2 = i + (i1 << 1);
      d1 = H[i2];
      d += d1 * X[i1];
      y_tmp[i1 + (i << 2)] = d1;
      x_tmp = i1 << 2;
      dv[i2] = ((H[i] * P[x_tmp] + H[i + 2] * P[x_tmp + 1]) +
                H[i + 4] * P[x_tmp + 2]) +
               H[i + 6] * P[x_tmp + 3];
    }
    e[i] = Z[i] - d;
  }
  for (int i{0}; i < 2; i++) {
    d = dv[i];
    d1 = dv[i + 2];
    d2 = dv[i + 4];
    d3 = dv[i + 6];
    for (int i1{0}; i1 < 2; i1++) {
      i2 = i1 << 2;
      x_tmp = i + (i1 << 1);
      x[x_tmp] = (((d * y_tmp[i2] + d1 * y_tmp[i2 + 1]) + d2 * y_tmp[i2 + 2]) +
                  d3 * y_tmp[i2 + 3]) +
                 R[x_tmp];
    }
  }
  if (std::abs(x[1]) > std::abs(x[0])) {
    r = x[0] / x[1];
    t = 1.0 / (r * x[3] - x[2]);
    b_tmp_idx_0 = x[3] / x[1] * t;
    b_tmp_idx_1 = -t;
    b_tmp_idx_2 = -x[2] / x[1] * t;
    t *= r;
  } else {
    r = x[1] / x[0];
    t = 1.0 / (x[3] - r * x[2]);
    b_tmp_idx_0 = x[3] / x[0] * t;
    b_tmp_idx_1 = -r * t;
    b_tmp_idx_2 = -x[2] / x[0] * t;
  }
  if (((e[0] * b_tmp_idx_0 + e[1] * b_tmp_idx_1) * e[0] +
           (e[0] * b_tmp_idx_2 + e[1] * t) * e[1] <=
       threshold) ||
      (update_flag == 0.0)) {
    double dv1[16];
    for (int i{0}; i < 4; i++) {
      d = P[i];
      d1 = P[i + 4];
      d2 = P[i + 8];
      d3 = P[i + 12];
      for (int i1{0}; i1 < 2; i1++) {
        i2 = i1 << 2;
        dv[i + i2] =
            ((d * y_tmp[i2] + d1 * y_tmp[i2 + 1]) + d2 * y_tmp[i2 + 2]) +
            d3 * y_tmp[i2 + 3];
      }
      d = dv[i + 4];
      d1 = dv[i];
      d2 = d1 * b_tmp_idx_0 + d * b_tmp_idx_1;
      d3 = d2;
      K[i] = d2;
      r = d2 * e[0];
      d2 = d1 * b_tmp_idx_2 + d * t;
      K[i + 4] = d2;
      r += d2 * e[1];
      X[i] += r;
      for (int i1{0}; i1 < 4; i1++) {
        i2 = i1 << 1;
        dv1[i + (i1 << 2)] = d3 * H[i2] + d2 * H[i2 + 1];
      }
      d = dv1[i];
      d1 = dv1[i + 4];
      d2 = dv1[i + 8];
      d3 = dv1[i + 12];
      for (int i1{0}; i1 < 4; i1++) {
        i2 = i1 << 2;
        x_tmp = i + i2;
        dv2[x_tmp] =
            P[x_tmp] -
            (((d * P[i2] + d1 * P[i2 + 1]) + d2 * P[i2 + 2]) + d3 * P[i2 + 3]);
      }
    }
    std::copy(&dv2[0], &dv2[16], &P[0]);
    num_not_updated = 0.0;
    update_flag = 1.0;
  } else {
    X[2] = 0.0;
    X[3] = 0.0;
    num_not_updated++;
    if (num_not_updated > 2.0) {
      update_flag = 0.0;
    }
  }
}

//
// File trailer for AlgoEKF.cpp
//
// [EOF]
//
