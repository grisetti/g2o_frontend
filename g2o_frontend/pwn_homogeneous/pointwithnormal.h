#ifndef _POINT_WITH_NORMAL_H_
#define _POINT_WITH_NORMAL_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#incude "homogeneous4f.h"

/** \typedef Vector6f
 *  \brief 6 elements float vector type.
 *
 *  The Vector6f type it's a 6x1 Eigen matrix of float elements.
*/
typedef Eigen::Matrix<float, 6, 1> Vector6f;

/** \typedef Matrix6f
 *  \brief 6x6 float matrix type.
 *
 *  The Matrix6f type it's a 6x6 Eigen matrix of float elements.
*/
typedef Eigen::Matrix<float, 6, 6> Matrix6f;

/** \typedef Matrix6fVector
 *  \brief Unsigned short matrix type.
 *
 *  The Matrix6fVector type it's a standard vector of Matrix6f.
*/
typedef std::vector<Matrix6f, Eigen::aligned_allocator<Matrix6f> > Matrix6fVector;

/** \struct PointWithNormal
 *  \brief This class defines the basic element of the system: a point with a normal.
 *
 *  A point with a normal is represented homogeneous vectors containing respectively the point
 *  coordinates and the normal. The homogeneous coordinates it's equal to 1 for the poit 
 *  coordinates and 0 for the normal. If the point coordinates or the normal are not defined 
 *  the first three elements of the vectors will be equal to zero. A point with normal can be 
 *  remapped through an isometry.
 */
struct PointWithNormal : {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
  /**
   *  Empty constructor.
   *  This constructor creates an invalid point with normal.
   */
  PointWithNormal() {}
  
  /**
   *  Constructor specifying only point coordinates.
   *  This constructor creates an point with normal filling its elements using the
   *  input point coordinate homogeneous vector.
   */
  PointWithNormal(HomogeneousPoint3f point_) {
    _point = point_;
  }

  /**
   *  Constructor specifying only normal coordinates.
   *  This constructor creates an point with normal filling its elements using the
   *  input normal homogeneous vector.
   */
  PointWithNormal(HomogeneousNormal3f normal_) {
    _normal = normal_;
  }
  
  /**
   *  Constructor specifying point coordinates and normal homogeneous vecotrs.
   *  This constructor creates an point with normal filling its elements using the
   *  input point coordinate and normal homogeneous vector.
   */
  PointWithNormal(HomogeneousPoint3f point_, HomogeneousNormal3f normal_) {
    _point = point_;
    _normal = normal_;
  }

  /**
   *  This method extract the point position coordinates.
   *  @return a 4 element float homogeneous vector containing the point position coordinates.
   */
  inline HomogeneousPoint3f point() const {
    return _point;
  }
  
  /**
   *  This method extract the point's normal.
   *  @return a 4 element float homogeneous vector containing the point's normal.
   */
  inline HomogeneousPoint3f normal() const {
    return _normal;
  }
  
  /**
   *  This method sets the point position coordinates elements using the given input vector. 
   *  @param p_ is the 4 elements float homogeneous vector used to fill the point position coordinates elements.
   */
  inline void setPoint(const HomogeneousPoint3f& point_) {
    _point = point_;
  }
  
  /**
   *  This method sets the point's normal elements using the given input vector. 
   *  @param p_ is the 4 elements float homogeneous vector used to fill the point's normal elements.
   */
  inline void setNormal(const HomogeneousNormal3f& normal_) {
    _normal = normal_;
  }
  
  /**
   *  This method check if the point with normal has point coordinates (0, 0, 0). 
   *  @return a bool value, true if the normal is valid, false otherwise.
   */
  inline bool isPointDefined() {
    return _point.head<3>().squaredNorm() > 0.0f;
  }

  /**
   *  This method check if the point with normal has a valid normal. 
   *  @return a bool value, true if the normal is valid, false otherwise.
   */
  inline bool isNormalDefined() {
    return _normal.head<3>().squaredNorm() > 0.0f;
  }

 protected:
   /**
   *  Variable containing the point coordinates.
   */
  HomogeneousPoint3f _point;
   /**
   *  Variable containing the normal components.
   */
  HomogeneousNormal3f _normal;
};

/**
 *  This method define the product operator between a point with normals and an isometry transformation.
 *  It allows to remap a point with normal given the remapping transformation.
 */
inline PointWithNormal operator*(const Eigen::Isometry3f& t, const PointWithNormal& pwn){
  PointWithNormal rpwn;
  rpwn.setPoint(t*pwn.point());
  rpwn.setNormal(t*pwn.nrmal());
  return rpwn;
}

/**
 *  Class for point with normals vector representation. 
 *  This class extends the standard vector class and implements the basic
 *  operation to manipulate vectors of PointWithNormal.
 */

class PointWithNormalVector : public std::vector<PointWithNormal, Eigen::aligned_allocator<PointWithNormal> > {
public:
  /**
   *  Constructor specifying vector dimension and filling point with normal.
   *  If the optional parameters are not given, this costructor creates a vector of point
   *  with normal with zero dimension. If the input parameters are given, then a vector
   *  of point with normal is created with the specified dimension and filled with point with
   *  normals of the type given in input.
   */
  PointWithNormalVector(size_t s=0, const PointWithNormal& p=PointWithNormal());

  /**
   *  This methods creates a depth image from the point with normal vector, given the camera matrix
   *  and the camera pose associated.
   *  @param depthImage is the output parameter where the depth image is stored.
   *  @param cameraMatrix is an input parameter representing the camera matrix.
   *  @param cameraPose is an input parameter representing the camera pose.
   *  @param dmax is an optional input parameter useful to cut off points over a certain depth value.
   */
  void toDepthImage(Eigen::MatrixXf& depthImage, 
		    const Eigen::Matrix3f& cameraMatrix, const Eigen::Isometry3f& cameraPose, 
		    float dmax = std::numeric_limits<float>::max()) const;

  /**
   *  This method creates a point with normal vector from a depth image, given the camera matrix
   *  and the camera pose associated.
   *  @param depthImage is an input parameter where the depth image is stored.
   *  @param cameraMatrix is an input parameter representing the camera matrix.
   *  @param cameraPose is an input parameter representing the camera pose.
   *  @param dmax is an optional input parameter useful to cut off points over a certain depth value.
   */
  void fromDepthImage(const Eigen::MatrixXf& depthImage, 
		      const Eigen::Matrix3f& cameraMatrix, const Eigen::Isometry3f& cameraPose, 
		      float dmax = std::numeric_limits<float>::max());

  /**
   *  This method creates an index image and a Z-buffer from the point with normal vector, 
   *  given the camera matrix and the camera pose associated. The index image contains the indices of
   *  the points inside the point with normal vector.
   *  @param indexImage is the output parameter where the indices are stored.
   *  @param zBuffer is the output parameter where Z-buffer is generated.
   *  @param cameraMatrix is an input parameter representing the camera matrix.
   *  @param cameraPose is an input parameter representing the camera pose.
   *  @param dmax is an optional input parameter useful to cut off points over a certain depth value.
   */
  void toIndexImage(Eigen::MatrixXi& indexImage, Eigen::MatrixXf& zBuffer, 
		    const Eigen::Matrix3f& cameraMatrix, const Eigen::Isometry3f& cameraPose, 
		    float dmax = std::numeric_limits<float>::max()) const;

  /**
   *  This method allows to save the point with normal vector on a .pwn file readable from
   *  the pwn_simpleViewer executable linux file.
   *  @param os is an input parameter representing an output stream to the file.
   *  @param step is an optional parameter whih is used to specify to save a point each step points.
   *  @param binary is an optional input parameter that depending on it's value write the file
   *  in two different manners.
   *  @return a bool value which is true if the file is correctly written, false otherwise.
   */
  bool save(std::ostream& os, int step=1, bool binary=false) const;
  
  /**
   *  This method allows to load the point with normal vector from a .pwn file readable from
   *  the pwn_simpleViewer executable linux file.
   *  @param is is an input parameter representing an input stream to the file.
   *  @return a bool value which is true if the file is correctly loaded, false otherwise.
   */
  bool load(std::istream& is);
  
  /**
   *  This method allows to save the point with normal vector on a .pwn file readable from
   *  the pwn_simpleViewer executable linux file.
   *  @param filename is an input parameter representing the name of the file to write.
   *  @param step is an optional parameter whih is used to specify to save a point each step points.
   *  @param binary is an optional input parameter that depending on it's value write the file
   *  in two different manners.
   *  @return a bool value which is true if the file is correctly written, false otherwise.
   */
  bool save(const char* filename, int step=1, bool binary=false) const;
  
  /**
   *  This method allows to load the point with normal vector from a .pwn file readable from
   *  the pwn_simpleViewer executable linux file.
   *  @param is is an input parameter representing the input file name.
   *  @return a bool value which is true if the file is correctly loaded, false otherwise.
   */
  bool load(const char* filename);

};

/**
 *  This method define the product operator between a point with normals vector and an 
 *  isometry transformation. It allows to remap the points of a point with normal vector 
 *  given the remapping transformation.
 */
PointWithNormalVector operator*(Eigen::Isometry3f t, const PointWithNormalVector& points);

/** \typedef PointWithNormalPtr
 *  \brief PointWithNormal pointer type.
 *
 *  The PointWithNormalPtr type it's a PointWithNormal pointer.
*/
typedef PointWithNormal* PointWithNormalPtr;

#endif
