#ifndef _MULTIVARIATE_VECTOR_FUNCTION_H_
#define _MULTIVARIATE_VECTOR_FUNCTION_H_

#include <Eigen/Core>
#include "gaussian.h"

template <typename Scalar_, int DomainDimensionAtCompileTime_, int CodomainDimensionAtCompileTime_> 
class AffineVectorFunction;

template <typename Scalar_, int DomainDimensionAtCompileTime_, int CodomainDimensionAtCompileTime_> 
class MultivariateVectorFunction {
public:
  typedef Scalar_ ScalarType;
  typedef Eigen::Matrix<Scalar_, DomainDimensionAtCompileTime_, 1> DomainType;
  typedef Eigen::Matrix<Scalar_, CodomainDimensionAtCompileTime_, 1> CodomainType;
  typedef Eigen::Matrix<Scalar_, CodomainDimensionAtCompileTime_, DomainDimensionAtCompileTime_> JacobianType;
  static const int DomainDimensionAtCompileTime=DomainDimensionAtCompileTime_;
  static const int CodomainDimensionAtCompileTime=CodomainDimensionAtCompileTime_;

  MultivariateVectorFunction(){
    if (DomainDimensionAtCompileTime_==Eigen::Dynamic && CodomainDimensionAtCompileTime_==Eigen::Dynamic){
      _domainDimension = 0;
      _codomainDimension = 0;
    } else if (DomainDimensionAtCompileTime_!=Eigen::Dynamic && CodomainDimensionAtCompileTime_==Eigen::Dynamic) {
      _domainDimension = DomainDimensionAtCompileTime_;
      _codomainDimension = 0;
    } else if (DomainDimensionAtCompileTime_==Eigen::Dynamic && CodomainDimensionAtCompileTime_!=Eigen::Dynamic) {
      _domainDimension = 0;
      _codomainDimension = CodomainDimensionAtCompileTime_;
    } else {
      _domainDimension = DomainDimensionAtCompileTime_;
      _codomainDimension = CodomainDimensionAtCompileTime_;
    }
  }

  MultivariateVectorFunction(int domainDimension_, int codomainDimension_) {
    if (DomainDimensionAtCompileTime_==Eigen::Dynamic && CodomainDimensionAtCompileTime_==Eigen::Dynamic){
      _domainDimension = domainDimension_;
      _codomainDimension = codomainDimension_;
    } else if (DomainDimensionAtCompileTime_!=Eigen::Dynamic && CodomainDimensionAtCompileTime_==Eigen::Dynamic) {
      assert (domainDimension_ == DomainDimensionAtCompileTime && "Error: domain dimension should match the dimension of the template parameters for fixed size object");
      _domainDimension = DomainDimensionAtCompileTime_;
      _codomainDimension = codomainDimension_;
    } else if (DomainDimensionAtCompileTime_==Eigen::Dynamic && CodomainDimensionAtCompileTime_!=Eigen::Dynamic) {
      assert (codomainDimension_ == CodomainDimensionAtCompileTime && "Error: codomain dimension should match the dimension of the template parameters for fixed size object");
      _domainDimension = domainDimension_;
      _codomainDimension = CodomainDimensionAtCompileTime_;
    } else {
      assert (domainDimension_ == DomainDimensionAtCompileTime && "Error: domain dimension should match the dimension of the template parameters for fixed size object");
      assert (codomainDimension_ == CodomainDimensionAtCompileTime && "Error: codomain dimension should match the dimension of the template parameters for fixed size object");
      _domainDimension = DomainDimensionAtCompileTime_;
      _codomainDimension = CodomainDimensionAtCompileTime_;
    }
  }

  virtual CodomainType operator()(const DomainType& x) const = 0;

 Gaussian<ScalarType, CodomainDimensionAtCompileTime_> operator()(const Gaussian<ScalarType, DomainDimensionAtCompileTime_ >
& g){
   CodomainType f = this->operator()(g.mean());
   JacobianType J = jacobian(g.mean());
   return Gaussian<ScalarType, CodomainDimensionAtCompileTime_>(f, J*g.covariance()*J.transpose());
  }

  virtual JacobianType jacobian(const DomainType& x) const {
    return numericJacobian(x);
  }

 
 
  AffineVectorFunction<Scalar_, DomainDimensionAtCompileTime_, CodomainDimensionAtCompileTime_> taylorExpansion(const DomainType& x) const;

  virtual ~MultivariateVectorFunction(){}

  inline int domainDimension() const {return DomainDimensionAtCompileTime==Eigen::Dynamic ? _domainDimension : DomainDimensionAtCompileTime; }
  inline int codomainDimension() const {return CodomainDimensionAtCompileTime==Eigen::Dynamic ? _codomainDimension : CodomainDimensionAtCompileTime; }
 
protected:
  JacobianType numericJacobian(const DomainType& x, ScalarType epsilon=1e-5) const  {
    JacobianType retval(_codomainDimension,_domainDimension);
    DomainType xup=x;
    DomainType xdown=x;
    ScalarType iEpsilon = .5/epsilon;
    for (int i=0; i<domainDimension(); i++){
      ScalarType xi=x(i);
      xup(i)+=epsilon;
      xdown(i)-=epsilon;
      retval.col(i)=iEpsilon*(this->operator()(xup)-this->operator()(xdown));
      xup(i)=xi;
      xdown(i)=xi;
    }
    return retval;
  }

  int _domainDimension;
  int _codomainDimension;
};


#endif
