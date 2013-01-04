#include "integralpointimage.h"

IntegralPointImage::IntegralPointImage(): Eigen::Matrix<PointAccumulator, Eigen::Dynamic, Eigen::Dynamic>(0.,0.){
}

void IntegralPointImage::compute(const Eigen::MatrixXi indices, const PointWithNormalVector& points){
  if (cols()!=indices.cols() || rows()!=indices.rows())
    resize(indices.rows(), indices.cols());
  clear();
  

  PointAccumulator* acc=data();
  const int* pointIndex = indices.data();
  int s = rows()*cols();
  // fill the accumulators with the points
  for (int i=0; i<s; i++, acc++, pointIndex++){
    if (*pointIndex<0)
      continue;
    const PointWithNormal& point=points[*pointIndex];
    acc->operator+=(point.head<3>());
  }

  // fill by column
  for (int c=0; c<cols(); c++){
    for (int r=1; r<rows(); r++){
      operator()(r,c) += operator()(r-1,c);
    }
  }

  // fill by column
  for (int c=1; c<cols(); c++){
    for (int r=0; r<rows(); r++){
      operator()(r,c) += operator()(r,c-1);
    }
  }

}

void IntegralPointImage::clear(){
  int s = rows()*cols();
  PointAccumulator* p=data();
  for (int i=0; i<s; i++, p++)
    p->clear();
}

inline int _clamp(int v, int min, int max){
  v=(v<min)?min:v;
  v=(v>max)?max:v;
  return v;
}

PointAccumulator IntegralPointImage::getRegion(int xmin, int xmax, int ymin, int ymax) const{
  PointAccumulator pa;
  if (! rows() || !cols())
    return pa;
  xmin=_clamp(xmin-1, 0, cols()-1);
  xmax=_clamp(xmax-1, 0, cols()-1);
  ymin=_clamp(ymin-1, 0, rows()-1);
  ymax=_clamp(ymax-1, 0, rows()-1);
  pa=operator()(ymax, xmax); //total
  pa+=operator()(ymin, xmin);  //upper right
  pa-=operator()(ymax, xmin);  //upper rectangle
  pa-=operator()(ymin, xmax);  //rightmost rectangle
  return pa;
}
