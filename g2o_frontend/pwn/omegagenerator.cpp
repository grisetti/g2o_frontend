#include "omegagenerator.h"
#include <omp.h>

using namespace Eigen;

void PointOmegaGenerator::compute(HomogeneousPoint3fOmegaVector &omegas, 
				  const HomogeneousPoint3fStatsVector &stats,
				  const HomogeneousNormal3fVector &imageNormals) {
  omegas.resize(stats.size());

#pragma omp parallel for
  for(size_t i = 0; i < stats.size(); i++) {
    const HomogeneousPoint3fStats &pointStats = stats[i];
    HomogeneousPoint3fOmega U = Matrix4f::Zero();
    U.block<3, 3>(0, 0) = pointStats.eigenVectors(); 
    if(imageNormals[i].squaredNorm()>0) {
      if(pointStats.curvature() < _curvatureThreshold)
	omegas[i] = U*_flatOmega*U.transpose();
      else {
	const Vector3f &eigenValues = pointStats.eigenValues();
	_nonFlatOmega.diagonal() = HomogeneousNormal3f(Vector3f(1.0f/eigenValues[0], 
								1.0f/eigenValues[1], 
								1.0f/eigenValues[2]));
	omegas[i] = U*_nonFlatOmega*U.transpose();
      }
    } else 
      omegas[i] = HomogeneousPoint3fOmega();
  }
}

void NormalOmegaGenerator::compute(HomogeneousPoint3fOmegaVector &omegas, 
				   const HomogeneousPoint3fStatsVector &stats,
				   const HomogeneousNormal3fVector &imageNormals) {
  omegas.resize(stats.size());

#pragma omp parallel for
  for(size_t i = 0; i < stats.size(); i++) {
    const HomogeneousPoint3fStats &pointStats = stats[i];
    if(imageNormals[i].squaredNorm()>0) {
      if(pointStats.curvature() < _curvatureThreshold)
	omegas[i] = _flatOmega;
      else 
	omegas[i] = _nonFlatOmega;
    } else 
      omegas[i] = HomogeneousPoint3fOmega();
  }
}
