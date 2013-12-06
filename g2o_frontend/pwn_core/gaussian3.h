#pragma once

#include "pwn_typedefs.h"
#include "homogeneousvector4f.h"

namespace pwn {

  class PinholePointProjector;
  
  class Gaussian3fVector : public TransformableVector<Gaussian3f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Gaussian3fVector(size_t s = 0, const Gaussian3f &p = Gaussian3f());

    void fromDepthImage(const DepthImage &depthImage, 
			const PinholePointProjector &pointProjector, 
			float dmax = std::numeric_limits<float>::max(), 
			float baseline = 0.075f, float alpha = 0.1f);

    void toPointAndNormalVector(PointVector &destPoints, NormalVector &destNormals, bool eraseNormals = false) const;

    template<typename OtherDerived>
      inline void transformInPlace(const OtherDerived &m) {
      const Eigen::Matrix4f t = m;
      const Eigen::Matrix3f rotation = t.block<3, 3>(0, 0);
      const Eigen::Vector3f translation = t.block<3, 1>(0, 3);
      for (size_t i = 0; i < size(); ++i) {
	at(i) = Gaussian3f(rotation * at(i).mean() + translation, rotation * at(i).covarianceMatrix() * rotation.transpose(), false);
      }
    }
  };

}
