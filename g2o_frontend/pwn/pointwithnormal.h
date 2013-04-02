#ifndef _POINT_WITH_NORMAL_H_
#define _POINT_WITH_NORMAL_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Dense>
//#include "homogeneousvector4f.h"

typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<float, 6, 6> Matrix6f;
typedef std::vector<Matrix6f, Eigen::aligned_allocator<Matrix6f> > Matrix6fVector;

/*This structure defines the basic element of the system: a point with a normal.
  A point with a normal is represented as a 6 vector, where the first three elements are the
  point position, and the last three elements are the normal. If the normal is not defined
  the last three elements are set to 0.
  A point with normal can be remapped through an isometry;
*/
struct PointWithNormal: public Vector6f {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  //! constructs a invalid point with a normal
  PointWithNormal() {}
  //! constructs a point with a normal from its 6d vector
  PointWithNormal(const Vector6f& v) {
    (*this) = v;
  }

  //! returns the point
  inline Eigen::Vector3f point() const {return Vector6f::head<3>();}
  //! returns the normal
  inline Eigen::Vector3f normal() const {return Vector6f::tail<3>();}
  inline void setPoint(const Eigen::Vector3f& p_) {head<3>() = p_;}
  inline void setNormal(const Eigen::Vector3f& n_) {tail<3>() = n_;}
  inline bool isNormalDefined() {return tail<3>().squaredNorm()>0.;}
};

inline PointWithNormal operator*(const Eigen::Isometry3f& t, const PointWithNormal& pwn){
  PointWithNormal rpwn;
  rpwn.head<3>()=t*pwn.head<3>();
  rpwn.tail<3>()=t.linear()*pwn.tail<3>();
  return rpwn;
}


class PointWithNormalVector: public std::vector<PointWithNormal, Eigen::aligned_allocator<PointWithNormal> > {
public:
  PointWithNormalVector(size_t s=0, const PointWithNormal& p=PointWithNormal());
  void toDepthImage(Eigen::MatrixXf& depthImage, 
		    const Eigen::Matrix3f& cameraMatrix, const Eigen::Isometry3f& cameraPose, 
		    float dmax = std::numeric_limits<float>::max()) const;

  void fromDepthImage(const Eigen::MatrixXf& depthImage, 
		      const Eigen::Matrix3f& cameraMatrix, const Eigen::Isometry3f& cameraPose, 
		      float dmax = std::numeric_limits<float>::max());

  void toIndexImage(Eigen::MatrixXi& indexImage, Eigen::MatrixXf& zBuffer, 
		    const Eigen::Matrix3f& cameraMatrix, const Eigen::Isometry3f& cameraPose, 
		    float dmax = std::numeric_limits<float>::max()) const;

  bool save(std::ostream & os, int step=1, bool binary=false) const;
  bool load(std::istream& is);
  bool save(const char* filename, int step=1, bool binary=false) const;
  bool load(const char* filename);

};

PointWithNormalVector operator*(Eigen::Isometry3f t, const PointWithNormalVector& points);

typedef PointWithNormal* PointWithNormalPtr;


#endif
