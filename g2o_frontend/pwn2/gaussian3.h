#ifndef _PWN_GAUSSIAN3_H_
#define _PWN_GAUSSIAN3_H_

#include "depthimage.h"
#include "homogeneousvector4f.h"
#include "../basemath/gaussian.h"

namespace pwn {

typedef Eigen::DiagonalMatrix<float, 3> Diagonal3f;
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
    const Eigen::Matrix4f t = m;
    const Eigen::Matrix3f rotation = t.block<3, 3>(0, 0);
    const Eigen::Vector3f translation = t.block<3, 1>(0, 3);
    for (size_t i = 0; i < size(); ++i) {
      at(i) = Gaussian3f(rotation * at(i).mean() + translation, rotation * at(i).covarianceMatrix() * rotation.transpose(), false);
    }
  }
};

}

#endif
