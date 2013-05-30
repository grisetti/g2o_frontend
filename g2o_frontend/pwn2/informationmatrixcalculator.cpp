#include "informationmatrixcalculator.h"
#include <omp.h>

using namespace Eigen;

namespace pwn {

void PointInformationMatrixCalculator::compute(InformationMatrixVector &informationMatrix,
					       const PointStatsVector &stats,
					       const NormalVector &imageNormals) {
  informationMatrix.resize(stats.size());

#pragma omp parallel for
  for(size_t i = 0; i < stats.size(); i++) {
    const PointStats &pointStats = stats[i];
    InformationMatrix U = Matrix4f::Zero();
    U.block<3, 3>(0, 0) = pointStats.eigenVectors(); 
    if(imageNormals[i].squaredNorm()>0) {
      if(pointStats.curvature() < _curvatureThreshold)
    informationMatrix[i] = U*_flatInformationMatrix*U.transpose();
      else {
	const Vector3f &eigenValues = pointStats.eigenValues();
    _nonFlatInformationMatrix.diagonal() = Normal(Vector3f(1.0f/eigenValues[0],
								1.0f/eigenValues[1], 
								1.0f/eigenValues[2]));
    informationMatrix[i] = U*_nonFlatInformationMatrix*U.transpose();
      }
    } else 
      informationMatrix[i] = InformationMatrix();
  }
}

void NormalInformationMatrixCalculator::compute(InformationMatrixVector &informationMatrix,
                   const PointStatsVector &stats,
                   const NormalVector &imageNormals) {
  informationMatrix.resize(stats.size());

#pragma omp parallel for
  for(size_t i = 0; i < stats.size(); i++) {
    const PointStats &pointStats = stats[i];
    if(imageNormals[i].squaredNorm()>0) {
      if(pointStats.curvature() < _curvatureThreshold)
    informationMatrix[i] = _flatInformationMatrix;
      else 
    informationMatrix[i] = _nonFlatInformationMatrix;
    } else 
      informationMatrix[i] = InformationMatrix();
 }
}

}
