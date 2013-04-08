#ifndef _PINHOLEPOINTPROJECTOR_H_
#define _PINHOLEPOINTPROJECTOR_H_

#include "pointprojector.h"

/**
 *  PointProjector class extension for operations on pinhole camera model points. 
 *  This class extend the PointProjector class allowing projection/unprojection
 *  of points using the pinhole camera model.
 */

class PinholePointProjector: public PointProjector {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   *  Empty constructor.
   *  This constructor creates a PinholePointProjector object setting the camera pose to the identity 
   *  while the maximum and minimum distance are imposed to be respectively 10.0 and 0.01 meters.
   */
  PinholePointProjector();
  
  /**
   *  Destructor.
   */
  virtual ~PinholePointProjector();
  
  /**
   *  Method that return the camera matrix.
   *  @return a reference to the camera matrix.
   */
  inline const Eigen::Matrix3f& cameraMatrix() const {return _cameraMatrix;}
  
  /**
   *  Virtual method that set the camera pose transformation to the one given in input.
   *  @param transform_ is the isometry used to update the camera pose transformation variable. 
   */
  virtual void setTransform(const Eigen::Isometry3f& transform_);
  
  /**
   *  Virtual method that set the camera matrix to the one given in input.
   *  @param transform_ is the 3x3 matrix used to update the camera matrix variable. 
   */
  virtual void setCameraMatrix(const Eigen::Matrix3f& cameraMatrix_);

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
  virtual void project(Eigen::MatrixXi& indexImage, 
		       Eigen::MatrixXf& depthImage, 
		       const HomogeneousPoint3fVector& points) const;

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
   *  @param y is an input parameter that contains the column of the matrix where the projected point
   *  falls.
   *  @param d is an input parameter that contains the depth value of the projected point.
   *  @param worldRadius is the input parameter containing the radius in the 3D euclidean space used
   *  to determine the side of the square region for the given point.
   *  @return an integer containing the value of the side of the square region that will be used to compute the 
   *  stats of the point.
   */
  void projectIntervals(Eigen::MatrixXi& intervalImage, 
			const Eigen::MatrixXf& depthImage, 
			float worldRadius) const;
  
  /**
   *  Virtual method that unprojects to the 3D euclidean space the points contained in the depthImage
   *  matrix. The indexImage matrix is used to place the unprojected inside the vector of points in a
   *  consistent position. This method stores the unprojected points in a vector of points.
   *  @param points is the output parameter containing the set of points unprojected to 3D euclidean space.
   *  @param indexImage is an input parameter containing indices. Each element of this matrix contains 
   *  the index where to place the corresponding point in the output vector of points.
   *  @param depthImage is an input parameter containing the depth values of the points.
   */
  virtual void unProject(HomogeneousPoint3fVector& points, 
			 Eigen::MatrixXi& indexImage, 
                         const Eigen::MatrixXf& depthImage) const;

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
  virtual int projectInterval(int x, int y, float d, float worldRadius) const;
  
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
  virtual bool project(int& x, int& y, float&f, const HomogeneousPoint3f& p) const;
  
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
  virtual bool unProject(HomogeneousPoint3f& p, int x, int y, float d) const;
 
  /**
   *  This variable contains the 3x3 camera matrix associated to the pinholePointProjector object.
   */
  Eigen::Matrix3f _cameraMatrix;  
  
  /**
   *  This variable contains the 4x3 matrix used to project the points to the image plane.
   *  It encodes also the transformation associated to the pinholePointProjector object.
   */ 
  Eigen::Matrix4f _KRt;
  
  /**
   *  This variable contains the 4x3 matrix used to unproject to the 3D euclidean space.
   *  It encodes also the transformation associated to the pinholePointProjector object.
   */
  Eigen::Matrix4f _iKRt;
  
  /**
   *  This method update all the matrices used for projection/unprojection operation.
   *  It is called each time the transformation or the camera matrix variable are modified,
   *  constructors included.
   *  @see project(). 
   */
  void _updateMatrices();

  /**
   *  Point projection method.
   *  @see project(). 
   */
  inline bool _project(int& x, int& y, float& d, const HomogeneousPoint3f& p) const {
    Eigen::Vector4f ip=_KRt*p;
    d=ip.coeff(2);
    if (d<_minDistance || d>_maxDistance)
      return false;
    ip*= (1./d);
    x=(int)round(ip.coeff(0));
    y=(int)round(ip.coeff(1)); 
    return true;
  }
  
  /**
   *  Point unprojection method.
   *  @see unProject(). 
   */
  inline  bool  _unProject(HomogeneousPoint3f& p, int x, int y, float d) const {
    if (d<_minDistance || d>_maxDistance)
      return false;
    p=_iKRt*Eigen::Vector4f(x*d,y*d,d,1.0);
    return true;
  }
  
  /**
   *  Square region's side projection method.
   *  @see projectInterval(). 
   */
  inline int _projectInterval(int, int, float d, float worldRadius) const {
    if (d<_minDistance || d>_maxDistance)
      return -1;
    Eigen::Matrix<float, 3,2> range;
    Eigen::Vector3f p=_cameraMatrix*Eigen::Vector3f(worldRadius,worldRadius,0);
    p*=(1./d);
    if (p.coeff(0)>p.coeff(1))
      return p.coeff(0);
    return p.coeff(1);
}

private:
  Eigen::Matrix3f _iK;
  Eigen::Matrix3f _KR;
  Eigen::Vector3f _Kt;
  Eigen::Matrix3f _iKR;
  Eigen::Vector3f _iKt;
};

#endif
