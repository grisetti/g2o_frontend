#include "omegagenerator.h"
#include <iostream>

using namespace Eigen;
using namespace std;

void PointOmegaGenerator::compute(HomogeneousPoint3fOmegaVector& omegas, 
				  HomogeneousPoint3fStatsVector& stats,
				  HomogeneousNormal3fVector& imageNormals) {
  HomogeneousPoint3fOmega U = Matrix4f::Zero();
  omegas.resize(stats.size());
  //cerr << "ci sono " << imageNormals.size() << endl;
  //cerr << "ci sono " << stats.size() << endl;
  for(size_t i = 0; i < stats.size(); i++) {
    HomogeneousPoint3fStats& pointStats = stats[i];
    U.block<3, 3>(0, 0) = pointStats.eigenVectors(); 
    if(imageNormals[i] != Vector4f::Zero()) {
      if(pointStats.curvature() < _curvatureThreshold)
	omegas[i] = U*_flatOmega*U.transpose();
      else {
	Vector3f eigenValues = pointStats.eigenValues();
	_nonFlatOmega.diagonal() = HomogeneousNormal3f(Vector3f(1.0f/eigenValues[0], 
								1.0f/eigenValues[1], 
								1.0f/eigenValues[2]));
	omegas[i] = U*_nonFlatOmega*U.transpose();
      }
    } else 
      omegas[i] = HomogeneousPoint3fOmega();
  }
  //cerr << "fine" << endl;
}

void NormalOmegaGenerator::compute(HomogeneousPoint3fOmegaVector& omegas, 
				   HomogeneousPoint3fStatsVector& stats,
				   HomogeneousNormal3fVector& imageNormals) {
  omegas.resize(stats.size());
  for(size_t i = 0; i < stats.size(); i++) {
    HomogeneousPoint3fStats& pointStats = stats[i];
    if(imageNormals[i] != Vector4f::Zero()) {
      if(pointStats.curvature() < _curvatureThreshold)
	omegas[i] = _flatOmega;
      else 
	omegas[i] = _nonFlatOmega;
    } else 
      omegas[i] = HomogeneousPoint3fOmega();
  }
}
