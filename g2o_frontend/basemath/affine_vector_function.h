#ifndef _AFFINE_VECTOR_FUNCTION_H_
#define _AFFINE_VECTOR_FUNCTION_H_

#include <Eigen/Core>

template <typename Scalar_, int DomainDimensionAtCompileTime_, int CodomainDimensionAtCompileTime_> 
class AffineVectorFunction : public MultivariateVectorFunction <Scalar_, DomainDimensionAtCompileTime_, CodomainDimensionAtCompileTime_> {
public:
  typedef Scalar_ ScalarType;
  typedef Eigen::Matrix<Scalar_, DomainDimensionAtCompileTime_, 1> DomainType;
  typedef Eigen::Matrix<Scalar_, CodomainDimensionAtCompileTime_, 1> CodomainType;
  typedef Eigen::Matrix<Scalar_, CodomainDimensionAtCompileTime_, DomainDimensionAtCompileTime_> JacobianType;
  typedef Eigen::Matrix<Scalar_, CodomainDimensionAtCompileTime_, DomainDimensionAtCompileTime_> MatrixType;

  AffineVectorFunction() : 
    MultivariateVectorFunction <ScalarType, DomainDimensionAtCompileTime_, CodomainDimensionAtCompileTime_>(),
    _A(this->codomainDimension(), this->domainDimension()),
    _b(this->codomainDimension())
  {}

  AffineVectorFunction(int domainDimension_, int codomainDimension_):
     MultivariateVectorFunction <ScalarType, DomainDimensionAtCompileTime_, CodomainDimensionAtCompileTime_>(),
     _A(codomainDimension_, domainDimension_),
    _b(codomainDimension_)
 {}

  inline const MatrixType& matrix() const {
    return _A;
  }

  inline void setMatrix(const MatrixType& A) {
    assert (A.rows()==codomainDimension() && A.cols == domainDimension() && "Sizes of the matrix shuould match the affine function parameters");
    _A = A;
  }

  inline const CodomainType& vector() const {
    return _b;
  }

  inline void setVector(const CodomainType& b) {
    assert(b.rows()==codomainDimension() && "Size of the vector shuould match the affine function parameters");
    _b = b;
  }

  virtual CodomainType operator()(const DomainType& x) const {
    return _A * x + _b;
  }

  virtual JacobianType jacobian(const DomainType&) const {
    return _A;
  }

protected:
  MatrixType _A;
  CodomainType _b;
};

template <typename Scalar_, int DomainDimensionAtCompileTime_, int CodomainDimensionAtCompileTime_> 
AffineVectorFunction<Scalar_, DomainDimensionAtCompileTime_, CodomainDimensionAtCompileTime_> 
MultivariateVectorFunction<Scalar_, DomainDimensionAtCompileTime_, CodomainDimensionAtCompileTime_>
::taylorExpansion(const MultivariateVectorFunction<Scalar_, DomainDimensionAtCompileTime_, CodomainDimensionAtCompileTime_>::DomainType& x) const {
  AffineVectorFunction<Scalar_,DomainDimensionAtCompileTime_, CodomainDimensionAtCompileTime_> affine(domainDimension(), codomainDimension());
  affine.setVector(this->operator()(x));
  affine.setMatrix(this->jacobian(x));
  return affine;
}


#endif
