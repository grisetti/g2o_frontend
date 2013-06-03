#ifndef _GAUSSIAN3_H_
#define _GAUSSIAN3_H_

#include "depthimage.h"
#include "homogeneousvector4f.h"
#include "../basemath/gaussian.h"

namespace pwn {

typedef struct Gaussian<float, 3> Gaussian3f;

class Gaussian3fVector : public TransformableVector<Gaussian3f> {
public:
  Gaussian3fVector(size_t s = 0, const Gaussian3f &p = Gaussian3f());

  void toDepthImage(DepthImage &depthImage, 
		    const Eigen::Matrix3f &cameraMatrix, const Eigen::Isometry3f &cameraPose, 
		    float dmax = std::numeric_limits<float>::max()) const;

  void fromDepthImage(const DepthImage &depthImage, 
		      const Eigen::Matrix3f &cameraMatrix, 
		      float dmax = std::numeric_limits<float>::max(), 
		      float baseline = 0.075f, float alpha = 0.1f);

  void toIndexImage(Eigen::MatrixXi &indexImage, DepthImage &depthImage, 
		    const Eigen::Matrix3f &cameraMatrix, const Eigen::Isometry3f &cameraPose, 
		    float dmax = std::numeric_limits<float>::max()) const;

  void toPointAndNormalVector(PointVector &destPoints, NormalVector &destNormals, bool eraseNormals = false) const;

  template<typename OtherDerived>
  inline void transformInPlace(const OtherDerived& m) {
    const Eigen::Matrix4f R4 = m;
    for (size_t i = 0; i < size(); ++i) {
      Gaussian3f transformed(R4.block<3, 3>(0, 0) * at(i).mean() + R4.block<3, 1>(0, 3), R4.block<3, 3>(0, 0) * at(i).covarianceMatrix() * R4.block<3, 3>(0, 0).transpose(), false);
      at(i) = transformed;
    }
  }
};

}

#endif
