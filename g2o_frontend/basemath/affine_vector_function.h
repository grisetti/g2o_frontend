#ifndef _AFFINE_VECTOR_FUNCTION_H_
#define _AFFINE_VECTOR_FUNCTION_H_

#include <Eigen/Core>
#include "gaussian.h"

template <typename DomainType_, typename CodomainType_> 
class AffineVectorFunction : public MultivariateVectorFunction <DomainType_, CodomainType_> {
public:
  typedef typename MultivariateVectorFunction <DomainType_, CodomainType_>::DomainScalarType DomainScalarType;
  typedef typename MultivariateVectorFunction <DomainType_, CodomainType_>::CodomainScalarType CodomainScalarType;

  typedef typename MultivariateVectorFunction <DomainType_, CodomainType_>::JacobianType MatrixType;
  typedef Eigen::Matrix< typename MultivariateVectorFunction <DomainType_, CodomainType_>::CodomainScalarType, CodomainType_::RowsAtCompileTime, 1> VectorType;

  inline const MatrixType& matrix() const {
    return _A;
  }

  inline void setMatrix(const MatrixType& A) {
    _A = A;
  }

  inline const VectorType& vector() const {
    return _b;
  }

  inline void setVector(const VectorType& b) {
    _b = b;
  }

  virtual typename MultivariateVectorFunction <DomainType_, CodomainType_>::CodomainType operator()(const typename MultivariateVectorFunction <DomainType_, CodomainType_>::DomainType& x) const {
    return _A * x + _b;
  }

  virtual typename MultivariateVectorFunction <DomainType_, CodomainType_>::JacobianType jacobian(const typename MultivariateVectorFunction <DomainType_, CodomainType_>::DomainType&) const {
    return _A;
  }

  virtual typename MultivariateVectorFunction <DomainType_, CodomainType_>::JacobianType jacobian() const {
    return _A;
  }

  Gaussian<CodomainScalarType, CodomainType_::RowsAtCompileTime> apply(const Gaussian<CodomainScalarType, DomainType_::RowsAtCompileTime >
& g){
    return Gaussian<CodomainScalarType, CodomainType_::RowsAtCompileTime>(_A*g.mean()+_b, _A*g.covariance()*_A.transpose());
  }

protected:
  MatrixType _A;
  VectorType _b;
};

template <typename DomainType_, typename CodomainType_> 
AffineVectorFunction<DomainType_,CodomainType_> MultivariateVectorFunction<DomainType_,CodomainType_>::taylorExpansion(const DomainType& x) const {
  AffineVectorFunction<DomainType_,CodomainType_> affine;
  affine.setVector(this->operator()(x));
  affine.setMatrix(this->jacobian(x));
  return affine;
}


#endif
