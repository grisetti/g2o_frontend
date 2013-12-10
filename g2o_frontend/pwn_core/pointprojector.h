#pragma once

#include "gaussian3.h"

namespace pwn {

  /** \class PointProjector pointprojector.h "pointprojector.h"
   *  \brief Base class interface for point projection/unprojection.
   *  
   *  Points in the 3D euclidean space can be projected to a subspace, which can be a plane,
   *  a cylinder, a sphere or whatever you can image. At the same way they can be unprojected
   *  from their projection subspace to the 3D euclidean space. This class provides a useful 
   *  interface that can be extended in order to build a specific PointProjector and then use it 
   *  to do the basic projection/unprojection operations on set of points.
   */
  class PointProjector {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     *  Empty constructor.
     *  This constructor creates a PointProjector object setting the pose transform to the identity, 
     *  the maximum and minimum distance are imposed to be respectively 10.0 and 0.01 meters,
     *  image rows and columns are setted to 0.
     */
    PointProjector();
    /**
     *  Destructor.
     */
    virtual ~PointProjector();
  
    /**
     *  Virtual method that return the pose transform.
     *  @return a constant reference to the pose transform.
     *  @see setTransform()
     */
    virtual inline const Eigen::Isometry3f &transform() const { return _transform; };  
    /**
     *  Virtual method that set the pose transform to the one given in input.
     *  @param transform_ is a constant reference to the isometry used to update the pose transform. 
     *  @see transform()
     */
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
    Eigen::Isometry3f _transform; /**< Transformation to apply to the camera pose in order to change the point of view from which the points are seen. */
  
    float _minDistance; /**< Minimum distance in meters for which all the points below its value are cutted. */
    float _maxDistance; /**< Maximum distance in meters for which all the points above its value are cutted. */

    int _imageRows; /**< Rows of the image to which the points will projected. */
    int _imageCols; /**< Columns of the image to which the points will projected. */
  };

}
