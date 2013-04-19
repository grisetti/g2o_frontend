#include "aligner.h"
#include <omp.h>
#include <iostream>

using namespace std;

float Linearizer::update() {
  // Variables initialization.
  _b = Vector6f::Zero();
  _H = Matrix6f::Zero();
  float error = 0.0f;
  int inliers = 0;
  HomogeneousPoint3fOmegaVector* pointOmegas = _aligner->currentPointOmegas();
  HomogeneousPoint3fOmegaVector* normalOmegas = _aligner->currentNormalOmegas();
  
//#pragma omp parallel for
  for(int i = 0; i < _aligner->numCorrespondences(); i++) {
    Correspondence& correspondence = _aligner->correspondences()->at(i);
    HomogeneousPoint3f referencePoint = _T*_aligner->referencePoints()->at(correspondence.referenceIndex);
    HomogeneousNormal3f referenceNormal = _T*_aligner->referenceNormals()->at(correspondence.referenceIndex);
    HomogeneousPoint3f currentPoint = _aligner->currentPoints()->at(correspondence.currentIndex);
    HomogeneousNormal3f currentNormal = _aligner->currentNormals()->at(correspondence.currentIndex);
    HomogeneousPoint3fOmega& pointOmega = pointOmegas->at(correspondence.currentIndex);
    HomogeneousPoint3fOmega& normalOmega = normalOmegas->at(correspondence.currentIndex);
    
    if (pointOmega.squaredNorm() == 0 || normalOmega.squaredNorm() == 0)
      continue;

    HomogeneousNormal3f pointError = referencePoint - currentPoint;
    HomogeneousNormal3f normalError = referenceNormal - currentNormal;
    float pointLocalError = pointError.transpose()*pointOmega*pointError;
    float normalLocalError = normalError.transpose()*normalOmega*normalError;
    float localError = pointLocalError + normalLocalError;
    if(localError > _inlierMaxChi2)
      continue;
    
    inliers++;
    error += localError;

    _computeHb_qt(_H, _b, 
		  referencePoint, referenceNormal, 
		  pointError, normalError, 
		  pointOmega, normalOmega);
  }
  _H.block<3,3>(3,0) = _H.block<3,3>(0,3).transpose();
  
  return error;
}
