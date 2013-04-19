#ifndef _SE3_MANIFOLD_H_
#define _SE3_MANIFOLD_H_

#include "smooth_manifold.h"

template <typename TransformType, int minimalDimension>
class SE3Manifold: public SmoothManifold<typename TransformType::Scalar, IsometryType, 6>{
  typedef typename VectorType::Scalar ScalarType;
  typedef Eigen::Matrix<Scalar, 6, 1> PerturbationVectorType;
  virtual oplus(const PerturbationVectorType &v){
    (&this)*=v2t(v);
  }
  virtual PerturbationVectorType ominus(const TransformType& v){
    return t2v(v*inverse());
  }
};

#endif
