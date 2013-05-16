#ifndef _PWN_SE3_PRIOR_H_
#define _PWN_SE3_PRIOR_H_

#include "g2o_frontend/basemath/bm_se3.h"

namespace pwn {

/**
   class that implements a simple prior on the transformation to be used in SE2 pose.
   The prior is defined as a gaussian distribution centered in a certain value (priorMean),
   and having a certain information matrix;
   
   The error induced by the prior is defined as 
   t2v(invT * priorMean),
   where invT s the *inverse* of the transformation we are looking for.

   Since we are operating on a manifold measurement space, the infromation matrix in the error
   space depends on the linearization point.
   
   To use this class:
   1) set the priorMean and the priorInformation to the desired values
   2) get the information of the error (omega), through the
      errorInformation(invT) function
   3) get the error and the jacobian to compute H and b;
*/

class SE3Prior {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  SE3Prior(const Eigen::Isometry3f& priorMean=Eigen::Isometry3f::Identity(), 
	   const Matrix6f& information=Matrix6f::Zero());
  inline void setMean(const Eigen::Isometry3f& priorMean_) {_priorMean = priorMean_;}
  inline void setInformation(const Matrix6f& priorInformation_) {_priorInformation = priorInformation_;}
  inline const Eigen::Isometry3f& mean() const {return _priorMean;}
  inline const Matrix6f& information() const {return _priorInformation;}

  //! computes the error of the prior at the inverse transform
  virtual Vector6f error(const Eigen::Isometry3f& invT) const;

  //! computes the jacobian of the error 
  Matrix6f jacobian(const Eigen::Isometry3f& invT) const;

  //! projects the information matrix of the prior in the error space
  Matrix6f errorInformation(const Eigen::Isometry3f& invT) const;

protected:
  //! computes the jacobian of the error w.r.t. the priorMean
  //! used internally to project the information matrix in the measurement space
  Matrix6f jacobianZ(const Eigen::Isometry3f& invT) const;
  
  mutable Eigen::Isometry3f  _priorMean;
  Matrix6f    _priorInformation;
};

}

#endif
