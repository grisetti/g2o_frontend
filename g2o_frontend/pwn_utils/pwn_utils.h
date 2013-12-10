#pragma once

#include <Eigen/Core>

using namespace std;
using namespace Eigen;

inline void isometry3f2d(Isometry3d &outputIsometry, const Isometry3f &inputIsometry) {
  for(int c = 0; c < 4; c++) {
    for(int r = 0; r < 3; r++) {
      outputIsometry.matrix()(r, c) = inputIsometry.matrix()(r, c);
    } 
  }
  outputIsometry.matrix().row(3) << 0.0l, 0.0l, 0.0l, 1.0l; 
}

inline void isometry3d2f(Isometry3f &outputIsometry, const Isometry3d &inputIsometry) {
  for(int c = 0; c < 4; c++) {
    for(int r = 0; r < 3; r++) {
      outputIsometry.matrix()(r, c) = inputIsometry.matrix()(r, c);
    } 
  }
  outputIsometry.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f; 
}

inline void xyzToQuat(Quaternionf &quaternion, float x, float y, float z) {
  float w = sqrtf(1.0f - x * x - y * y - z * z);
  quaternion = Quaternionf(w, x, y, z);
}
