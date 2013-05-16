#ifndef _SMOOTH_MANIFOLD_H_
#define _SMOOTH_MANIFOLD_H_

template <typename InternalType_, int PerturbationDimensionAtCompileTime>
class SmoothManifold : public InternalType{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef InternalType_ InternalType;
  SmoothManifold(): InternalType(){}
  SmoothManifold(const InternalType& other) : InternalType(other){}

  typedef typename InternalType_::Scalar ScalarType;
  typedef Eigen::Matrix<Scalar, PerturbationDimensionAtCompileTime, 1> PerturbationVectorType;
  virtual void oplus(const PerturbationVectorType&) = 0;
  virtual PerturbationVectorType ominus(const InternalType& other) const = 0;
};

enum AssociationType = {AssociateLeft=0x1, AssociateRight=0x2};
template <typename InternalType_, int PerturbationDimensionAtCompileTime, AssociationType AssociationMode_>
class  GroupManifold: public SmoothManifold <InternalType, PerturbationDimensionAtCompileTime> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static const int AssociationMode=AssociationMode_;
  typedef typename InternalType::Scalar ScalarType;
  typedef Eigen::Matrix<Scalar, PerturbationDimensionAtCompileTime, 1> PerturbationVectorType;

  GroupManifold(): SmoothManifold <InternalType, PerturbationDimensionAtCompileTime>(){}
  GroupManifold(const InternalType& other) : SmoothManifold <InternalType, PerturbationDimensionAtCompileTime>(other){}

  virtual PerturbationVectorType toVector() const  = 0;
  virtual void fromVector(const PerturbationVectorType&) const = 0;

  virtual void oplus(const PerturbationVectorType& v){
    GroupManifold<InternalType_,PerturbationDimensionAtCompileTime, AssociationMode_> delta;
    delta.fromVector(v);
    if (AssociationMode_==AssociateLeft){
      (*this)*=delta;
    } else if (AssociationMode_==AssociateRight){
      (*this)=delta*(*this);
    }
  }
  virtual PerturbationVectorType ominus(const InternalType& other) const {
    GroupManifold<InternalType_,PerturbationDimensionAtCompileTime, AssociationMode_> delta = other.inverse();
    if (AssociationMode_==AssociateLeft){
      delta = delta*(*this);
    } else if (AssociationMode_==AssociateRight){
      delta = (*this)*delta;
    }
    return delta.toVector();
  }
};


#endif
