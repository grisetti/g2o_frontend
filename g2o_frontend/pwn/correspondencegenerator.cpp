#include "correspondencegenerator.h"
#include <iostream>

void CorrespondenceGenerator::compute(CorrespondenceVector &correspondences,
				      const HomogeneousPoint3fVector &referencePoints, const HomogeneousPoint3fVector &currentPoints,
				      const HomogeneousNormal3fVector &referenceNormals, const HomogeneousNormal3fVector &currentNormals,
				      Eigen::MatrixXi &referenceIndexImage, const Eigen::MatrixXi &currentIndexImage,
				      const HomogeneousPoint3fStatsVector &referenceStats, const HomogeneousPoint3fStatsVector &currentStats,
				      Eigen::Isometry3f &T) {
  int correspondenceIndex = 0;
  _numCorrespondences = correspondenceIndex;
  if((int)correspondences.size() != referenceIndexImage.rows()*referenceIndexImage.cols())
    correspondences.resize(referenceIndexImage.rows()*referenceIndexImage.cols());
  
  for (int c = 0; c < referenceIndexImage.cols(); c++) {
    for (int r = 0; r < referenceIndexImage.rows(); r++) {
      int referenceIndex = referenceIndexImage(r, c);
      int currentIndex = currentIndexImage(r, c);
      if (referenceIndex < 0 || currentIndex < 0)
	continue;
      
      HomogeneousPoint3f referencePoint = T*referencePoints.at(referenceIndex);
      HomogeneousNormal3f referenceNormal = T*referenceNormals.at(referenceIndex);
      HomogeneousPoint3f currentPoint = currentPoints.at(currentIndex);
      HomogeneousNormal3f currentNormal = currentNormals.at(currentIndex);      
      HomogeneousNormal3f pointsDistance = currentPoint - referencePoint;
      if (pointsDistance.squaredNorm() > _squaredThreshold)
        continue;      
      if (currentNormal.dot(referenceNormal) < _inlierNormalAngularThreshold) 
        continue;

      float referenceCurvature = referenceStats[referenceIndex].curvature();
      float currentCurvature = currentStats[currentIndex].curvature();
      if (referenceCurvature < _flatCurvatureThreshold)
	referenceCurvature = _flatCurvatureThreshold;
      if (currentCurvature < _flatCurvatureThreshold)
	currentCurvature = _flatCurvatureThreshold;
      
      float logRatio = log(referenceCurvature + 1e-5) - log(currentCurvature + 1e-5);
      if (fabs(logRatio) > _inlierCurvatureRatioThreshold)
     	continue;
      
      correspondences[correspondenceIndex].referenceIndex = referenceIndex;
      correspondences[correspondenceIndex].currentIndex = currentIndex;
      
      correspondenceIndex++;
    }

    _numCorrespondences = correspondenceIndex;

    for (size_t i = _numCorrespondences; i < correspondences.size(); i++)
      correspondences[i] = Correspondence();
  }
}
