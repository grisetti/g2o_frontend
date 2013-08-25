#ifndef _PWN_CYLINDRICALPOINTPROJECTOR_H_
#define _PWN_CYLINDRICALPOINTPROJECTOR_H_
#include "g2o_frontend/boss_logger/eigen_boss_plugin.h" 
#include "g2o_frontend/boss/object_data.h"
#include "g2o_frontend/boss/identifiable.h"

#include "pointprojector.h"

namespace pwn {

/**
 *  PointProjector class extension for operations on pinhole camera model points. 
 *  This class extend the PointProjector class allowing projection/unprojection
 *  of points using the pinhole camera model.
 */

  class CylindricalPointProjector: public PointProjector {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     *  Empty constructor.
     *  This constructor creates a CylindricalPointProjector object setting the camera pose to the identity 
     *  while the maximum and minimum distance are imposed to be respectively 10.0 and 0.01 meters.
     */
    CylindricalPointProjector(int id=-1, boss::IdContext* context = 0);
  
    /**
     *  Destructor.
     */
    virtual ~CylindricalPointProjector();
  
    /**
     *  Virtual method that set the camera pose transformation to the one given in input.
     *  @param transform_ is the isometry used to update the camera pose transformation variable. 
     */
    virtual void setTransform(const Eigen::Isometry3f &transform_) {   
      _transform = transform_;
      _updateMatrices();
    }

    inline float angularFov() const {return _angularFov;} 
    inline void setAngularFov(float angularFov_)  { _angularFov = angularFov_; _updateMatrices();} 
    inline float angularResolution() const {return _angularResolution;} 
    inline void setAngularResolution(float angularResolution_)  { _angularResolution = angularResolution_; _updateMatrices();}
    inline float verticalFocalLenght() const {return _verticalFocalLenght;} 
    inline void setVerticalFocalLenght(float verticalFocalLenght_)  { _verticalFocalLenght = verticalFocalLenght_; _updateMatrices();} 
    inline float verticalCenter() const {return _verticalCenter;} 
    inline void setVerticalCenter(float verticalCenter_)  { _verticalCenter = verticalCenter_; _updateMatrices();} 

 
    /**
     *  Virtual method that projects a given set of homogeneous points from the 3D euclidean space to 
     *  to the image plane of the associated pinholePointProjector. This method stores the result
     *  in two matrix given in input. The projected points that falls out of the matrix or that are 
     *  behind other points are not stored in the matrices.
     *  @param indexImage is an output parameter containing indices. Each element of this matrix contains 
     *  the index of the corresponding point in the input vector of points.
     *  @param depthImage is an output parameter containing the depth values of the projected points.
     *  @param points is the input parameter containing the set of points to project.
     */
    virtual void project(Eigen::MatrixXi &indexImage, 
			 Eigen::MatrixXf &depthImage, 
			 const PointVector& points) const;

    /**
     *  Virtual method that projects the sides of the square regions used to compute the stats of the given point
     *  on a matrix given the depth image and the radius used to compute the sides.
     *  @param intervalImage is the image containing the sides of the square regions that will be used to compute
     *  the stats of the 3D points.
     *  @param depthImage is an input parameter containing the depth values of the points.
     *  @param worldRadius is the input parameter containing the radius in the 3D euclidean space used
     *  to determine the side of the square region for the given point.
     *  @param x is an input parameter that contains the row of the matrix where the projected point
     *  falls.
     */
    void projectIntervals(Eigen::MatrixXi &intervalImage, 
			  const Eigen::MatrixXf &depthImage, 
			  const float worldRadius,
			  const bool blackBorders=false) const;
  
    /**
     *  Virtual method that unprojects to the 3D euclidean space the points contained in the depthImage
     *  matrix. The indexImage matrix is used to place the unprojected inside the vector of points in a
     *  consistent position. This method stores the unprojected points in a vector of points.
     *  @param points is the output parameter containing the set of points unprojected to 3D euclidean space.
     *  @param indexImage is an output parameter containing indices. Each element of this matrix contains 
     *  the index where to place the corresponding point in the output vector of points.
     *  @param depthImage is an input parameter containing the depth values of the points.
     */
    virtual void unProject(PointVector &points, 
			   Eigen::MatrixXi &indexImage, 
			   const Eigen::MatrixXf &depthImage) const;

    virtual void unProject(PointVector &points,
			   Gaussian3fVector &gaussians,
			   Eigen::MatrixXi &indexImage,
			   const Eigen::MatrixXf &depthImage) const;

    /**
     *  Virtual method that projects the side of the square region used to compute the stats of the given point. 
     *  @param x is an input parameter that contains the row of the matrix where the projected point
     *  falls.
     *  @param y is an input parameter that contains the column of the matrix where the projected point
     *  falls.
     *  @param d is an input parameter that contains the depth value of the projected point.
     *  @param worldRadius is the input parameter containing the radius in the 3D euclidean space used
     *  to determine the side of the square region for the given point.
     *  @return an integer containing the value of the side of the square region that will be used to compute the 
     *  stats of the point.
     */
    virtual inline int projectInterval(const int x, const int y, const float d, const float worldRadius) const;
  
    /**
     *  Virtual method that projects a given homogeneous point from the 3D euclidean space to the iamge plane. 
     *  This method stores the resulting depth value and its matrix coordinates in the first three parameters.
     *  @param x is an output parameter that will contain the row of the matrix where the projected point
     *  falls.
     *  @param y is an output parameter that will contain the column of the matrix where the projected point
     *  falls.
     *  @param f is an output parameter that will contain the depth value of the projected point.
     *  @param p is the input parameter containing the 3D point to project.
     *  @return a bool value which is true if the depth value of the input point falls in the range
     *  defined by the minimum and maximum distance variables values, false otherwise.
     */
    virtual bool project(int &x, int &y, float &f, const Point &p) const;
  
    /**
     *  Virtual method that unprojects a given point defined through its depth value and its matrix coordinates to the 3D 
     *  euclidean space. This method stores the resulting 3D point in the first parameter.
     *  @param p is the output parameter containing the unprojected point.
     *  @param x is an input parameter that contains the row of the matrix where the input point
     *  falls.
     *  @param y is an input parameter that contains the column of the matrix where the input point
     *  falls.
     *  @param d is an input parameter that contains the depth value of the input point.
     *  @return a bool value which is true if the depth value of the input point falls in the range
     *  defined by the minimum and maximum distance variables values, false otherwise. 
     */
    virtual bool unProject(Point &p, const int x, const int y, const float d) const;
 
    /**
     *  This method update all the matrices used for projection/unprojection operation.
     *  It is called each time the transformation or the camera matrix variable are modified,
     *  constructors included.
     */
    void _updateMatrices();

    /**
     *  Point projection method.
     *  @see project(). 
     */
    inline bool _project(int &x, int &y, float &d, const Point &p) const {
      // point in camera coordinates;
      Eigen::Vector4f cp=_iT*p;
      d = sqrt(cp.x()*cp.x()+cp.z()*cp.z());
      if (d>_maxDistance || d<_minDistance)
	return false;
      float theta = atan2(cp.x(),cp.z());
      if (fabs(theta>_angularFov))
	return false;
      x = (int)(_angularResolution*theta+_angularCenter);
      y = (int)(cp.y()*_verticalFocalLenght/d + _verticalCenter);
      return true;
    }
  
    /**
     *  Point unprojection method.
     *  @see unProject(). 
     */
    inline bool _unProject(Point& p, const int x_, const int y_, const float d) const {
      if (d<_minDistance || d>_maxDistance)
	return false;
      float theta = _inverseAngularResolution*(x_-_angularCenter);
      if (fabs(theta>_angularFov))
	return false;
      float x=sin(theta)*d;
      float z=cos(theta)*d;
      float y=(y_-_verticalCenter)*d*_inverseVerticalFocalLenght;
      p=_transform*Eigen::Vector3f(x,y,z);
      return true;
    }
  
    /**
     *  Square region's side projection method.
     *  @see projectInterval(). 
     */
    inline int _projectInterval(const int, const int, const float d, const float worldRadius) const {
      // point in camera coordinates;
      if (d>_maxDistance || d<_minDistance)
	return -1;
      Eigen::Vector4f cp=Eigen::Vector4f(worldRadius,worldRadius,d,1);
      float theta = atan2(cp.x(),cp.z());
      int x = (int)(_angularResolution*theta);
      int y = (int)(cp.y()*_verticalFocalLenght/d);
      return std::max(x,y);
    }

    virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);
  
  protected:
    float _angularFov; // field of view
    float _angularCenter;
    float _verticalFocalLenght;  // focal lenght on the y
    float _inverseVerticalFocalLenght;  // focal lenght on the y
    float _verticalCenter;  // focal lenght on the y
    float _angularResolution; // radians per pixel;
    float _inverseAngularResolution; // pixels per radians;
    Eigen::Isometry3f _iT;
  };

}

#endif
