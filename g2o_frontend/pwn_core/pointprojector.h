#pragma once

#include "gaussian3.h"

namespace pwn {

  class PointProjector {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    PointProjector();
    virtual ~PointProjector();
  
    virtual inline const Eigen::Isometry3f &transform() const { return _transform; };  
    virtual inline void setTransform(const Eigen::Isometry3f &transform_) { 
      _transform = transform_; 
      _transform.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    }

    inline float minDistance() const { return _minDistance; }
    inline void setMinDistance(const float minDistance_) { _minDistance = minDistance_; }

    inline float maxDistance() const { return _maxDistance; }
    inline void setMaxDistance(const float maxDistance_) { _maxDistance = maxDistance_; }

    inline void setImageSize(const int rows, const int cols) {
      _imageRows = rows;
      _imageCols = cols;
    }    
    inline int imageRows() const { return _imageRows;}
    inline int imageCols() const { return _imageCols;}
  
    virtual void project(IntImage &indexImage, 
			 DepthImage &depthImage, 
			 const PointVector &points) const;
    virtual void unProject(PointVector &points,
			   IntImage &indexImage, 
			   const DepthImage &depthImage) const;
    virtual void unProject(PointVector &points,
			   Gaussian3fVector &gaussians,
			   IntImage &indexImage,
			   const DepthImage &depthImage) const;
    virtual void projectIntervals(IntImage &intervalImage, 
				  const DepthImage &depthImage, 
				  const float worldRadius,
				  const bool blackBorders = false) const;

    virtual inline bool project(int &, int &, float &, const Point &) const { return false; }
    virtual inline bool unProject(Point&, const int, const int, const float) const { return false; }
    virtual inline int projectInterval(const int, const int, const float, const float) const { return 0; }

    virtual void scale(float scalingFactor) = 0;
    
  protected:
    Eigen::Isometry3f _transform;
  
    float _minDistance; 
    float _maxDistance;

    int _imageRows;
    int _imageCols;
  };

}
