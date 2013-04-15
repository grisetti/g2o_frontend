#ifndef _GAUSSIAN3_H_
#define _GAUSSIAN3_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include "pointwithnormal.h"
#include "../basemath/gaussian.h"

typedef struct Gaussian<float,3> Gaussian3f;

inline Gaussian3f operator*(const Eigen::Isometry3f& t, const Gaussian3f& g){
  return Gaussian3f(t*g.mean(), t.linear() * g.covarianceMatrix() * t.linear().transpose(), false);
};


class Gaussian3fVector: public std::vector<Gaussian3f, Eigen::aligned_allocator<Gaussian3f> > {
public:
  Gaussian3fVector(size_t s=0, const Gaussian3f& p=Gaussian3f());
  void toDepthImage(Eigen::MatrixXf& depthImage, 
		    const Eigen::Matrix3f& cameraMatrix, const Eigen::Isometry3f& cameraPose, 
		    float dmax = std::numeric_limits<float>::max()) const;

  void fromDepthImage(const Eigen::MatrixXf& depthImage, 
		      const Eigen::Matrix3f& cameraMatrix, 
		      float dmax = std::numeric_limits<float>::max(), 
		      float baseline = 0.075, float alpha=0.1);

  void toIndexImage(Eigen::MatrixXi& indexImage, Eigen::MatrixXf& zBuffer, 
		    const Eigen::Matrix3f& cameraMatrix, const Eigen::Isometry3f& cameraPose, 
		    float dmax = std::numeric_limits<float>::max()) const;

  void toPointWithNormalVector(PointWithNormalVector& dest, bool eraseNormals = false) const;
  //bool save(const char* filename, int step=1) const;
  //bool load(const char* filename);

};

#endif
