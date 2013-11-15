#include "informationmatrixcalculator.h"
#include <omp.h>

using namespace Eigen;

namespace pwn {

  void InformationMatrixCalculator::serialize(boss::ObjectData& data, boss::IdContext& context) {
    Identifiable::serialize(data,context);
    _flatInformationMatrix.toBOSS(data, "flatInformationMatrix");
    _nonFlatInformationMatrix.toBOSS(data, "nonflatInformationMatrix");
  }
  
  void InformationMatrixCalculator::deserialize(boss::ObjectData& data, boss::IdContext& context){
    Identifiable::deserialize(data,context);
    _flatInformationMatrix.fromBOSS(data, "flatInformationMatrix");
    _nonFlatInformationMatrix.fromBOSS(data, "nonflatInformationMatrix");
  }

  void PointInformationMatrixCalculator::compute(InformationMatrixVector &informationMatrix,
						 const StatsVector &statsVector,
						 const NormalVector &imageNormals) {
    informationMatrix.resize(statsVector.size());
    

#pragma omp parallel for
    for(size_t i = 0; i < statsVector.size(); i++) {
      const Stats &stats = statsVector[i];
      InformationMatrix U = Matrix4f::Zero();
      U.block<3, 3>(0, 0) = stats.eigenVectors(); 
      if(imageNormals[i].squaredNorm()>0) {
	if(stats.curvature() < _curvatureThreshold)
	  informationMatrix[i] = U * _flatInformationMatrix * U.transpose();
	else {
	  const Vector3f &eigenValues = stats.eigenValues();
	  _nonFlatInformationMatrix.diagonal() = Normal(Vector3f(1.0f/eigenValues[0],
								 1.0f/eigenValues[1], 
								 1.0f/eigenValues[2]));
	  informationMatrix[i] = U * _nonFlatInformationMatrix * U.transpose();
	}
      } else 
	informationMatrix[i] = InformationMatrix();
    }
  }


  void PointInformationMatrixCalculator::serialize(boss::ObjectData& data, boss::IdContext& context) {
    InformationMatrixCalculator::serialize(data,context);
  }
  
  void PointInformationMatrixCalculator::deserialize(boss::ObjectData& data, boss::IdContext& context){
    InformationMatrixCalculator::deserialize(data,context);
  }


  void NormalInformationMatrixCalculator::compute(InformationMatrixVector &informationMatrix,
						  const StatsVector &statsVector,
						  const NormalVector &imageNormals) {
    informationMatrix.resize(statsVector.size());

#pragma omp parallel for
    for(size_t i = 0; i < statsVector.size(); i++) {
      const Stats &stats = statsVector[i];
      if(imageNormals[i].squaredNorm()>0) {
	if(stats.curvature() < _curvatureThreshold)
	  informationMatrix[i] = _flatInformationMatrix;
	else 
	  informationMatrix[i] = _nonFlatInformationMatrix;
      } else 
	informationMatrix[i] = InformationMatrix();
    }
  }

  void NormalInformationMatrixCalculator::serialize(boss::ObjectData& data, boss::IdContext& context) {
    InformationMatrixCalculator::serialize(data,context);
  }
  
  void NormalInformationMatrixCalculator::deserialize(boss::ObjectData& data, boss::IdContext& context){
    InformationMatrixCalculator::deserialize(data,context);
  }

  BOSS_REGISTER_CLASS(PointInformationMatrixCalculator);
  BOSS_REGISTER_CLASS(NormalInformationMatrixCalculator);
  
}
