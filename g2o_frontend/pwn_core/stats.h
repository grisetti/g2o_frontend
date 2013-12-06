#pragma once

#include "homogeneousvector4f.h"

namespace pwn {

  struct Stats : public Eigen::Matrix4f {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    inline Stats() {
      _n = 0;
      setIdentity();
      _eigenValues.setZero();
      _curvatureComputed = false;
      _curvature = 1.0f;	
    }

    inline int n() { return _n; }
    inline void setN(const int n_) { _n = n_; }

    inline Point mean() { return block<4, 1>(0, 3); }
    inline void setMean(const Point mean_) { block<4, 1>(0, 3) = mean_; }

    inline const Eigen::Vector3f& eigenValues() const { return _eigenValues; }
    inline void setEigenValues(const Eigen::Vector3f &eigenValues_)  { _eigenValues = eigenValues_; }

    inline Eigen::Matrix3f eigenVectors() const { return block<3, 3>(0, 0); }
    inline void  setEigenVectors(const Eigen::Matrix3f &eigenVectors_)  { block<3, 3>(0, 0) = eigenVectors_; }

    inline float curvature() const {
      if(!_curvatureComputed)
	_curvature = _eigenValues(0) / (_eigenValues(0) + _eigenValues(1) + _eigenValues(2) + 1e-9);
      _curvatureComputed = true;
      return _curvature;
    }
    inline void setCurvature(float curvature_) {
      _curvature = curvature_;
      _curvatureComputed = true;
    }
  
  protected:  	
    int _n;
    Eigen::Vector3f _eigenValues;
    mutable bool _curvatureComputed;
    mutable float _curvature;
  };

  class StatsVector : public TransformableVector<Stats> {
  public: 
    template<typename OtherDerived>
      inline void transformInPlace(const OtherDerived &m) {
      const Eigen::Matrix4f R4 = m;
      for (size_t i = 0; i < size(); ++i) {
	at(i).block<4, 4>(0, 0) = R4 * at(i).block<4, 4>(0, 0);
      }
    }
  };

}
