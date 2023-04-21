// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
