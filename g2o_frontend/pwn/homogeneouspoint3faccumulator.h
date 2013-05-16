#ifndef _HOMOGENEOUSPOINT3FACCUMULATOR_H_
#define _HOMOGENEOUSPOINT3FACCUMULATOR_H_

#include "homogeneousvector4f.h"

/** \struct HomogeneousPoint3fAccumulator
 *  \brief Point accumulator class.
 *
 *  This class allows to accumulate points doing the sum and the squared sum of 
 *  the points, it also keep track of the number of points accumulated. It is mainly
 *  used to compute the covariance matrix of the accumulated points along with
 *  their mean. Points can be added or removed simply using the plus and minus 
 *  operators. 
 */

struct HomogeneousPoint3fAccumulator {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
  /**
   *  Empty constructor.
   *  This constructor creates an HomogeneousPoint3fAccumulator object with the sum
   *  and squared sum setted to zero.
   */
  inline HomogeneousPoint3fAccumulator() { clear(); }
  
  /**
   *  This method reset the sum and squared sum variables to the zero value.
   */
  inline void clear() {
    _sum.setZero();
    _squaredSum.setZero();
  }
  
  /**
   *  This method define the plus operator between two HomogeneousPoint3fAccumulator  
   *  allowing to merge two HomogeneousPoint3fAccumulator.
   */
  inline void operator +=(const HomogeneousPoint3fAccumulator &pa) {
    _sum += pa._sum;
    _squaredSum += pa._squaredSum;
  }
  
  /**
   *  This method define the minus operator between two HomogeneousPoint3fAccumulator  
   *  allowing to subtract one HomogeneousPoint3fAccumulator from one other.
   */
  inline void operator -=(const HomogeneousPoint3fAccumulator &pa) {
    _sum -= pa._sum;
    _squaredSum -= pa._squaredSum;
  }

  /**
   *  This method define the plus operator between an HomogeneousPoint3fAccumulator  
   *  and HomogeneousPoint3f allowing to add a point to the accumulator.
   */
  inline void operator +=(const HomogeneousPoint3f &v) {
    _sum += (Eigen::Vector4f&)v;
    _squaredSum += v * v.transpose();
  }

  /**
   *  This method computes the mean of the accumulated points.
   *  @return an homogeneous 3D point which is the mean of the accumulated 
   *  points.
   */
  inline HomogeneousPoint3f mean() const { 
    const float &d = _sum.coeff(3, 0);
    if(d)
      return HomogeneousPoint3f(_sum * (1.0f/d));
    return HomogeneousPoint3f::Zero();
  }

  /**
   *  This method computes the covariance matrix using the accumulated points.
   *  @return a 4x4 matrix represnting the covariance matrix of the accumulated 
   *  points.
   */
  inline Eigen::Matrix4f covariance() const { 
    float d = _sum.coeff(3, 0);
    if(d) {
      d = 1.0f / d;
      Eigen::Vector4f mean_ = _sum*d;
      return _squaredSum * d - mean_ * mean_.transpose();
    }
    return Eigen::Matrix4f::Zero(); 
  }

  /**
   *  This method extract the sum of the accumulated points.
   *  @return a reference to 4 element vector representing the sum of the 
   *  accumulated points.
   */
  inline const Eigen::Vector4f& sum() const { return _sum; }
  
  /**
   *  This method extract the squared sum of the accumulated points.
   *  @return a reference to 4x4 matrix representing the squared sum of the 
   *  accumulated points.
   */
  inline const Eigen::Matrix4f& squaredSum() const { return _squaredSum; }
  
  /**
   *  This method computes the number of points accumulated.
   *  @return an integer representing the number of points accumulated.
   */
  inline int n() const { return _sum.coeff(3, 0); }

protected:
  /**
   *  Variable containing the sum of the points accumulated.
   */
  Eigen::Vector4f _sum;
  
  /**
   *  Variable containing the squared sum of the points accumulated.
   */
  Eigen::Matrix4f  _squaredSum;
};

#endif
