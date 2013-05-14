#include "pointintegralimage.h"
#include <omp.h>

PointIntegralImage::PointIntegralImage() : Eigen::Matrix<PointAccumulator, Eigen::Dynamic, Eigen::Dynamic>(0, 0) {}

void PointIntegralImage::compute(const Eigen::MatrixXi &indices, const PointVector &points) {
  if (cols() != indices.cols() || rows() != indices.rows())
    resize(indices.rows(), indices.cols());
  clear();
  
  PointAccumulator *acc = data();
  const int *pointIndex = indices.data();
  int s = rows() * cols();
  // fill the accumulators with the points
  for (int i=0; i<s; i++, acc++, pointIndex++){
    if (*pointIndex<0)
      continue;
    const Point& point = points[*pointIndex];
    acc->operator += (point);
  }

  // fill by column
  #pragma omp parallel for
  for (int c=0; c<cols(); c++){
    for (int r=1; r<rows(); r++){
      coeffRef(r,c) += coeffRef(r-1,c);
    }
  }

  // fill by row
  #pragma omp parallel for
  for (int r=0; r<rows(); r++){
    for (int c=1; c<cols(); c++){
      coeffRef(r,c) += coeffRef(r,c-1);
    }
  }
}

void PointIntegralImage::clear() {
  int s = rows() * cols();
  PointAccumulator *p = data();
  for (int i=0; i<s; i++, p++)
    p->clear();
}

inline int _clamp(int v, int min, int max) {
  v = (v < min) ? min : v;
  v = (v > max) ? max : v;
  return v;
}

PointAccumulator PointIntegralImage::getRegion(int xmin, int xmax, int ymin, int ymax) const {
  PointAccumulator pa;
  if (! rows() || !cols())
    return pa;
  xmin = _clamp(xmin-1, 0, rows()-1);
  xmax = _clamp(xmax-1, 0, rows()-1);
  ymin = _clamp(ymin-1, 0, cols()-1);
  ymax = _clamp(ymax-1, 0, cols()-1);
  pa = coeffRef(xmax,ymax); //total
  pa += coeffRef(xmin,ymin);  //upper right
  pa -= coeffRef(xmin, ymax);  //upper rectangle
  pa -= coeffRef(xmax, ymin);  //rightmost rectangle
  return pa;
}
