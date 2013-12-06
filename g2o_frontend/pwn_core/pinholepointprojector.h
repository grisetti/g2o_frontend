#pragma once

#include "pointprojector.h"

namespace pwn {

  class PinholePointProjector : public PointProjector {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PinholePointProjector();  
    virtual ~PinholePointProjector();
  
    virtual inline void setTransform(const Eigen::Isometry3f &transform_) {   
      _transform = transform_;
      _updateMatrices();
    }

    inline const Eigen::Matrix3f& cameraMatrix() const { return _cameraMatrix; }  
    inline void setCameraMatrix(const Eigen::Matrix3f &cameraMatrix_) {
      _cameraMatrix = cameraMatrix_;
      _updateMatrices();
    }
    inline const Eigen::Matrix3f& inverseCameraMatrix() const { return _iK; }

    inline float baseline() const { return _baseline; }
    inline void setBaseline(float baseline_) { _baseline = baseline_; }

    inline float alpha() const { return _alpha; }  
    inline void setAlpha(float alpha_) { _alpha = alpha_; }

    virtual void project(IntImage &indexImage, 
			 DepthImage &depthImage, 
			 const PointVector& points);
    virtual void unProject(PointVector &points, 
			   IntImage &indexImage, 
			   const DepthImage &depthImage) const;
    virtual void unProject(PointVector &points,
			   Gaussian3fVector &gaussians,
			   IntImage &indexImage,
			   const DepthImage &depthImage) const;
    void projectIntervals(IntImage &intervalImage, 
			  const DepthImage &depthImage, 
			  const float worldRadius,
			  const bool blackBorders = false) const;

    virtual inline bool project(int &x, int &y, float &f, const Point &p) const { return _project(x, y, f, p); }
    virtual inline bool unProject(Point &p, const int x, const int y, const float d) const { return _unProject(p, x, y, d); }
    virtual inline int projectInterval(const int x, const int y, const float d, const float worldRadius) const { return _projectInterval(x, y, d, worldRadius); }  

    virtual void scale(float scalingFactor);
    
  protected:  
    inline bool _project(int &x, int &y, float &d, const Point &p) const {
      Eigen::Vector4f ip = _KRt * p;
      d = ip.coeff(2);
      if(d < _minDistance || d > _maxDistance)
	return false;
      ip *= (1.0f / d);
      x = (int)round(ip.coeff(0));
      y = (int)round(ip.coeff(1)); 
      return true;
    }
  
    inline bool _unProject(Point &p, const int x, const int y, const float d) const {
      if(d < _minDistance || d > _maxDistance)
	return false;
      p = _iKRt * Eigen::Vector4f(x * d,y * d, d, 1.0f);
      return true;
    }
  
    inline int _projectInterval(const int, const int, const float d, const float worldRadius) const {
      if (d < _minDistance || d > _maxDistance)
	return -1;
      Eigen::Matrix<float, 3, 2> range;
      Eigen::Vector3f p = _cameraMatrix * Eigen::Vector3f(worldRadius, worldRadius, 0);
      p *= (1.0f / d);
      if(p.coeff(0) > p.coeff(1))
	return p.coeff(0);
      return p.coeff(1);
    }

    void _updateMatrices();
  
    float _baseline;
    float _alpha;

    Eigen::Matrix3f _cameraMatrix;    
    Eigen::Matrix4f _KRt;  
    Eigen::Matrix4f _iKRt;

    Eigen::Matrix3f _iK;
    Eigen::Matrix3f _KR;
    Eigen::Vector3f _Kt;
    Eigen::Matrix3f _iKR;
    Eigen::Vector3f _iKt;
  };

}
