#ifndef _PWN_POINTPROJECTOR_H_
#define _PWN_POINTPROJECTOR_H_

#include "g2o_frontend/boss_map/eigen_boss_plugin.h" 
#include "g2o_frontend/boss/object_data.h"
#include "g2o_frontend/boss/identifiable.h"


#include "pwn_typedefs.h"
#include "homogeneousvector4f.h"
#include "depthimage.h"
#include "gaussian3.h"

namespace pwn {

  /**
   *  Generic class for projection/unprojection operations of points. 
   *  Points in the 3D euclidean space can be projected to a subspace, which can be a plane,
   *  a cylinder, a sphere or whatever you can image. At the same way they can be unprojected
   *  from their projection subspace to the 3D euclidean space. This class can be extended in
   *  order to build specific PointProjector and then use it to do the basic 
   *  projection/unprojection operations on set of points.   
   */

  class PointProjector : public boss::Identifiable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     *  Empty constructor.
     *  This constructor creates a PointProjector object setting the camera pose to the identity 
     *  while the maximum and minimum distance are imposed to be respectively 10.0 and 0.01 meters.
     */
    PointProjector(int id=-1, boss::IdContext* context = 0);
  
    /**
     *  Destructor.
     */
    virtual ~PointProjector();
  
    /**
     *  Virtual method that return the camera pose transformation.
     *  @return a reference to the camera pose transformation.
     */
    virtual inline const Eigen::Isometry3f &transform() const { return _transform; };
  
    /**
     *  Virtual method that set the camera pose transformation to the one given in input.
     *  @param transform_ is the isometry used to update the camera pose transformation variable. 
     */
    virtual inline void setTransform(const Eigen::Isometry3f &transform_) { _transform = transform_; _transform.matrix().row(3) << 0,0,0,1;}

    /**
     *  Method that return the minimum distance value.
     *  @return the minimum distance value.
     */
    inline float minDistance() const { return _minDistance; }
  
    /**
     *  Method that set the minimum distance value to the one given in input.
     *  @param minDistance_ is the value used to update the minimum distance variable. 
     */
    inline void setMinDistance(const float minDistance_) { _minDistance = minDistance_; }
  
    /**
     *  Method that return the maximum distance value.
     *  @return the maximum distance value.
     */
    inline float maxDistance() const { return _maxDistance; }
  
    /**
     *  Method that set the maximum distance value to the one given in input.
     *  @param maxDistance_ is the value used to update the maximum distance variable. 
     */
    inline void setMaxDistance(const float maxDistance_) { _maxDistance = maxDistance_; }
  
    /**
     *  Virtual method that projects a given set of homogeneous points from the 3D euclidean space to 
     *  a destination space defined by the user extending this class. This method stores the result
     *  in two matrix given in input. The projected points that falls out of the matrix or that are 
     *  behind other points are not stored in the matrices.
     *  @param indexImage is an output parameter containing indices. Each element of this matrix contains 
     *  the index of the corresponding point in the input vector of points.
     *  @param depthImage is an output parameter containing the depth values of the projected points.
     *  @param points is the input parameter containing the set of points to project.
     */
    virtual void project(IntImage &indexImage, 
			 Eigen::MatrixXf &depthImage, 
			 const PointVector &points);

    /**
     *  Virtual method that projects on an image the sides of the square regions that will be used to compute
     *  the stats of the 3D points.
     *  @param intervalImage is the output parameter containing the sides of the square regions that will be
     *  used to compute the stats of the 3D oints.
     *  @param depthImage is an output parameter containing the depth values of the projected points.
     *  @param worldRadius is the input parameter containing radius in the 3D euclidean space used to determine
     *  the side of the square regions.
     */
    virtual void projectIntervals(IntImage &intervalImage, 
				  const Eigen::MatrixXf &depthImage, 
				  const float worldRadius,
				  const bool blackBorders=false) const;

    /**
     *  Virtual method that unprojects to the 3D euclidean space the points contained in the depthImage
     *  matrix. The indexImage matrix is used to place the unprojected inside the vector of points in a
     *  consistent position. The unprojection operation is specific for the projection space defined by the class  
     *  extending this class. This method stores the unprojected points in a vector of points.
     *  @param points is the output parameter containing the set of points unprojected to 3D euclidean space.
     *  @param indexImage is an output parameter containing indices. Each element of this matrix contains 
     *  the index where to place the corresponding point in the output vector of points.
     *  @param depthImage is an input parameter containing the depth values of the points.
     */
    virtual void unProject(PointVector &points,
			   IntImage &indexImage, 
			   const Eigen::MatrixXf &depthImage) const;

    virtual void unProject(PointVector &points,
			   Gaussian3fVector &gaussians,
			   IntImage &indexImage,
			   const Eigen::MatrixXf &depthImage) const;

    /**
     *  Pure virtual method that has to be implemented in all classes that extend this class. 
     *  It projects a given homogeneous point from the 3D euclidean space to 
     *  a destination space defined by the user extending this class. This method stores the resulting
     *  depth value and its matrix coordinates in the first three parameters.
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
     *  Pure virtual method that has to be implemented in all classes that extend this class. 
     *  It projects the side of the square region used to compute the stats of the given point. 
     *  @param x is an input parameter that contains the row of the matrix where the projected point
     *  falls.
     *  @param y is an input parameter that contains the column of the matrix where the projected point
     *  falls.
     *  @param d is an input parameter that contains the depth value of the projected point.
     *  @param worldRadius is the input parameter containing the radius in the 3D euclidean space used
     *  to determine the side of the square region for the given point.
     *  @return an integer containing the value of the side of the square region used to compute the 
     *  stats of the point.
     */
    virtual int projectInterval(const int x, const int y, const float d, const float worldRadius) const;

    /**
     *  Pure virtual method that has to be implemented in all classes that extend this class. 
     *  It unprojects a given point defined through its depth value and its matrix coordinates to the 3D 
     *  euclidean space. The unprojection operation is specific for the projection space defined by the class  
     *  extending this class. This method stores the resulting 3D point in the first parameter.
     *  @param p is the output parameter containing the unprojected point.
     *  @param x is an input parameter that contains the row of the matrix where the input point
     *  falls.
     *  @param y is an input parameter that contains the column of the matrix where the input point
     *  falls.
     *  @param d is an input parameter that contains the depth value of the input point.
     *  @return a bool value which is true if the depth value of the input point falls in the range
     *  defined by the minimum and maximum distance variables values, false otherwise. 
     */
    virtual bool unProject(Point& p, const int x, const int y, const float d) const;


    /**
       computes the new camera parameters that zoom/unzoom the current view
     */
    virtual void scale(float scalingFactor) = 0;

    /**serialization functions*/
    virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);

    inline void setImageSize(int rows, int cols) {
      _imageRows = rows;
      _imageCols = cols;
    }
    
    inline int imageRows() const {return _imageRows;}
    inline int imageCols() const {return _imageCols;}
    
  protected:
    /**
     *  This variable contains the transformation to apply to the camera pose in order to change,
     *  if desired, the point of view from which the points are seen.
     */
    Eigen::Isometry3f _transform;
  
    /**
     *  This variable is used to set a minimum distance in meters for which all the points under its value 
     *  are not considered and therefore cut off.
     */
    float _minDistance; 

    /**
     *  This variable is used to set a maximum distance in meters for which all the points over its value 
     *  are not considered and therefore cut off.
     */
    float _maxDistance;

    int _imageRows;
    int _imageCols;
  };

}

#endif
