#pragma once

#include "pointprojector.h"

namespace pwn {

  class CylindricalPointProjector : virtual public PointProjector {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    CylindricalPointProjector(); 
    virtual ~CylindricalPointProjector() {}
  
    virtual void setTransform(const Eigen::Isometry3f &transform_) {   
      _transform = transform_;
      _updateMatrices();
    }

    inline float angularFov() const { return _angularFov; } 
    inline void setAngularFov(float angularFov_)  { 
      _angularFov = angularFov_; 
      _updateMatrices();
    } 
    
    inline float angularResolution() const { return _angularResolution; } 
    inline void setAngularResolution(float angularResolution_)  { 
      _angularResolution = angularResolution_; 
      _updateMatrices();
    }
    
    inline float verticalFocalLenght() const { return _verticalFocalLenght; } 
    inline void setVerticalFocalLenght(float verticalFocalLenght_)  { 
      _verticalFocalLenght = verticalFocalLenght_; 
      _updateMatrices();
    } 
    
    inline float verticalCenter() const { return _verticalCenter; } 
    inline void setVerticalCenter(float verticalCenter_)  { 
      _verticalCenter = verticalCenter_; 
      _updateMatrices();
    } 

    virtual void project(IntImage &indexImage, 
			 DepthImage &depthImage, 
			 const PointVector& points) ;
    virtual void unProject(PointVector &points, 
			   IntImage &indexImage, 
			   const DepthImage &depthImage) const;
    virtual void unProject(PointVector &points,
			   Gaussian3fVector &gaussians,
			   IntImage &indexImage,
			   const DepthImage &depthImage) const;
    virtual void projectIntervals(IntImage &intervalImage, 
				  const DepthImage &depthImage, 
				  const float worldRadius) const;  
  
    virtual inline bool project(int &x, int &y, float &f, const Point &p) const { return _project(x, y, f, p); }
    virtual inline bool unProject(Point &p, const int x, const int y, const float d) const { return _unProject(p, x, y, d); }
    virtual inline int projectInterval(const int x, const int y, const float d, const float worldRadius) const { return _projectInterval(x, y, d, worldRadius); }
 
    inline bool _project(int &x, int &y, float &d, const Point &p) const {
      // Point in camera coordinates;
      Eigen::Vector4f cp = _iT*p;
      d = sqrt(cp.x() * cp.x() + cp.z() * cp.z());
      if(d > _maxDistance || d < _minDistance) {
	return false;
      }
      float theta = atan2(cp.x(), cp.z());
      if(fabs(theta > _angularFov)) {
	return false;
      }
      x = (int)(_angularResolution * theta + _angularCenter);
      y = (int)(cp.y() * _verticalFocalLenght/d + _verticalCenter);
      return true;
    } 

    inline bool _unProject(Point &p, const int x_, const int y_, const float d) const {
      if(d < _minDistance || d > _maxDistance)
	return false;
      float theta = _inverseAngularResolution * (x_ - _angularCenter);
      if(fabs(theta > _angularFov))
	return false;
      float x = sin(theta) * d;
      float z = cos(theta) * d;
      float y = (y_ - _verticalCenter) * d * _inverseVerticalFocalLenght;
      p = _transform * Eigen::Vector3f(x, y, z);
      return true;
    }
  
    inline int _projectInterval(const int, const int, const float d, const float worldRadius) const {
      // Point in camera coordinates;
      if(d > _maxDistance || d < _minDistance)
	return -1;
      Eigen::Vector4f cp = Eigen::Vector4f(worldRadius, worldRadius, d, 1);
      float theta = atan2(cp.x(), cp.z());
      int x = (int)(_angularResolution * theta);
      int y = (int)(cp.y() *_verticalFocalLenght / d);
      return std::max(x, y);
    }

    void _updateMatrices();
    
    virtual void scale(float scalingFactor);

  protected:
    float _angularFov; // Field of view
    float _angularCenter;
    float _verticalFocalLenght;  // Focal lenght on the y
    float _inverseVerticalFocalLenght;  // Focal lenght on the y
    float _verticalCenter;  // Focal lenght on the y
    float _angularResolution; // Radians per pixel;
    float _inverseAngularResolution; // Pixels per radians;
    Eigen::Isometry3f _iT;
  };

}
