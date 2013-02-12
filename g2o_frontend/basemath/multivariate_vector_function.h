#ifndef _MULTIVARIATE_VECTOR_FUNCTION_H_
#define _MULTIVARIATE_VECTOR_FUNCTION_H_

#include <Eigen/Core>

template <typename DomainType_, typename CodomainType_> 
class MultivariateVectorFunction {
public:
  typedef DomainType_ DomainType;
  typedef CodomainType_ CodomainType;
  typedef typename DomainType::Scalar DomainScalarType;
  typedef typename CodomainType::Scalar CodomainScalarType;
  typedef Eigen::Matrix<CodomainScalarType, CodomainType::ColsAtCompileTime, DomainType::ColsAtCompileTime> JacobianType;
  virtual CodomainType operator()(const DomainType& x) const = 0;
  JacobianType numeric_jacobian(const DomainType& x, DomainScalarType epsilon=1e-5) {
    JacobianType retval;
    DomainType xup=x;
    DomainType xdown=x;
    CodomainScalarType iEpsilon = .5/epsilon;
    for (int i=0; i<DomainType::ColsAtCompileTime; i++){
      DomainScalarType xi=x(i);
      xup(i)+=epsilon;
      xdown(i)-=epsilon;
      retval.col(i)=iEpsilon*(this->operator()(xup)-this->operator()(xdown));
      xup(i)=xi;
      xdown(i)=xi;
    }
    return retval;
  }
  JacobianType numeric_jacobian(DomainScalarType epsilon=1e-5) const {
    DomainType x;
    x.setZero();
    return numeric_jacobian(x,epsilon);
  }
};


#endif
