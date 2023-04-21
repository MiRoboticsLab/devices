//
// File: AlgoEKF.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 21-Apr-2023 15:42:45
//

#ifndef ALGOEKF_H
#define ALGOEKF_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
class AlgoEKF {
public:
  AlgoEKF();
  ~AlgoEKF();
  void EKF_init(double dis_init, double theta_init, double Q_var, double R_var);
  void EKF_pridect(double dt);
  void EKF_update(double dis, double theta, double threshold);
  double update_flag;
  double num_not_updated;
  double X[4];
  double A[16];
  double Z[2];
  double K[8];
  double H[8];
  double P[16];
  double Q[16];
  double R[4];
  int initialized;
};

#endif
//
// File trailer for AlgoEKF.h
//
// [EOF]
//
