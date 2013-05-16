#ifndef _VECTOR_MANIFOLD_H_
#define _VECTOR_MANIFOLD_H_

#include "smooth_manifold.h"

template <typename VectorType>
class VectorManifold: public SmoothManifold<typename VectorType::Scalar, VectorType, VectorType::RowsAtCompileTime>{
  typedef typename VectorType::Scalar ScalarType;
  typedef Eigen::Matrix<Scalar, PerturbationDimensionAtCompileTime, 1> PerturbationVectorType;
  void oplus(const PerturbationVectorType &v){
    (*this)+=v;
  }
  PerturbationVectorType ominus(const VectorType& v){
    return (*this)-v;
  }
};

#endif
