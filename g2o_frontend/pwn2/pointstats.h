#ifndef _POINTSTATS_H_
#define _POINTSTATS_H_

#include "homogeneousvector4f.h"

namespace pwn {

/** \struct HomogeneousPoint3fStats
 *  \brief Class for 3D points stats representation.
 *
 *  This class extends the Eigen Matrix4f class allowing to represent some stats associated to
 *  a 3D point. In particular it is able to store the eigenvalues and eigenvectors of the covariance
 *  matrix associated to a point along with its curvature.
 */
struct PointStats : public Eigen::Matrix4f {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  /**
   *  Empty constructor.
   *  This constructor creates an HomogeneousPoint3fStats object filling the matrix with zeros.
   */
  inline PointStats() : Eigen::Matrix4f() {
    _n = 0;
    setZero();
    _mean = Point::Zero();
    _curvatureComputed = false;
    _curvature = 1.0f;	
  }

  /** \typedef Base
   *  \brief 4 elements float vector type.
   *
   *  The Base type it's an Eigen Vector4f.
   */
  typedef Eigen::Matrix4f Base;

  /**
   *  Constructor using an Eigen MatrixBase object.
   *  This constructor creates an HomogeneousPoint3fStats object using the constructor of the
   *  Eigen Matrix4f class using as input the given generic Eigen MatrixBase object. Then it impose 
   *  the fourth row of the matrix  to be equal to zero.
   *  @param other is the Eigen MatrixBase object used to fill the matrix.
   */
  template<typename OtherDerived>
  inline PointStats(const Eigen::MatrixBase<OtherDerived> &other) : Eigen::Matrix4f(other){
    block<1, 4>(3, 0).setZero();
    _curvatureComputed = false;
  }

  /**
   *  This method define the assignment operator between a generic Eigen
   *  MatrixBase object and an HomogeneousPoint3fStats object.
   */
  template<typename OtherDerived>
  inline PointStats& operator = (const Eigen::MatrixBase<OtherDerived> &other) {
    this->Base::operator = (other);    
    block<1, 4>(3, 0).setZero();
    return *this;
  }

  /**
   *  This method extract the eigenvalues of the covariance matrix of the point associated to 
   *  the HomogeneousPoint3fStats object.
   *  @return a 3 element vector containing the eigenvalues extracted.
   */
  inline Eigen::Vector3f eigenValues() const { return block<3, 1>(0, 3); }

  /**
   *  This method extract the eigenvectors of the covariance matrix of the point associated to 
   *  the HomogeneousPoint3fStats object.
   *  @return a 3x3 matrix containing the eigenvalues extracted, where each column represent an eigenvector.
   */
  inline Eigen::Matrix3f eigenVectors() const { return block<3, 3>(0, 0); }
  
  template<typename OtherDerived>
  inline PointStats transform(const Eigen::MatrixBase<OtherDerived> &other) const {
    PointStats s(other*(*this));
    s.col(3) = col(3);
    return s;
  }

  template<typename OtherDerived>
  inline PointStats& transformInPlace(const Eigen::MatrixBase<OtherDerived> &other) const {
    PointStats s(other*(*this));
    s.col(3) = col(3);
    *this = s;
    return *this;
  }

  /**
   *  This method computes the curvature of the point associated to the HomogeneousPoint3fStats object.
   *  The curvature values range between 0 and 1. When the curvature is near to zero means that the point
   *  is lying on a flat surface (a plane), when it is near to 1 then the point is lying on a surface with
   *  an high curvature (a corner).
   *  @return a float value between 0 and 1 representing the curvature.
   */
  inline float curvature() const {
    if(!_curvatureComputed)
      _curvature = coeffRef(0, 3) / (coeffRef(0, 3) + coeffRef(1, 3) + coeffRef(2, 3) + 1e-9);
    _curvatureComputed = true;
    return _curvature;
  }
  
  inline int n() { return _n; }
  inline void setN(const int n_) { _n = n_; }
  inline Point mean() { return _mean; }
  inline void setMean(const Point mean_) { _mean = mean_; }

 protected:  	
  int _n;
  Point _mean;
  mutable bool  _curvatureComputed;
  mutable float _curvature;
};


 template<typename OtherDerived>
   inline PointStats operator *(const Eigen::MatrixBase<OtherDerived> &other, const PointStats& stats) {
   const Eigen::Matrix3f& R = other.block(3,3,0,0);
   PointStats s=stats;
   s.block<3,3>(0,0) = R * stats.block<3,3>(0,0);
   return s;
 }

/** \typedef HomogeneousPoint3fStatsVector
 *  \brief TransformableVector of HomogeneousPoint3fStats.
 *
 *  The HomogeneousPoint3fStatsVector type it's a TrasformableVector of stats associated to 3D points.
*/
typedef TransformableVector<PointStats> PointStatsVector;

}

#endif
