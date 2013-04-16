#include "correspondencegenerator.h"
#include <iostream>

using namespace std;

void CorrespondenceGenerator::compute(CorrespondenceVector &correspondences,
				      const HomogeneousPoint3fVector &referencePoints, const HomogeneousPoint3fVector &currentPoints,
				      const HomogeneousNormal3fVector &referenceNormals, const HomogeneousNormal3fVector &currentNormals,
				      Eigen::MatrixXi &referenceIndexImage, const Eigen::MatrixXi &currentIndexImage,
				      const HomogeneousPoint3fStatsVector &referenceStats, const HomogeneousPoint3fStatsVector &currentStats,
				      Eigen::Isometry3f &T) {
  int correspondenceIndex = 0;
  correspondences.resize(referencePoints.size());
  for (int c = 0; c < referenceIndexImage.cols(); c++) {
    for (int r = 0; r < referenceIndexImage.rows(); r++) {
      int &referenceIndex = referenceIndexImage(r, c);
      int currentIndex = currentIndexImage(r, c);
      if (referenceIndex < 0 || currentIndex < 0)
	continue;
      
      // compare euclidean distance and angle for the normals in the remapped frame
      HomogeneousPoint3f referencePoint = T*referencePoints.at(referenceIndex);
      HomogeneousNormal3f referenceNormal = T*referenceNormals.at(referenceIndex);
      HomogeneousPoint3f currentPoint = currentPoints.at(currentIndex);
      HomogeneousNormal3f currentNormal = currentNormals.at(currentIndex);
      if (referenceNormal.squaredNorm() == 0 || currentNormal.squaredNorm() == 0) {
	referenceIndex = -referenceIndex;
	continue;
      }

      HomogeneousPoint3f pointsDistance = currentPoint - referencePoint;
      if (pointsDistance.squaredNorm() > _squaredThreshold) {
	referenceIndex = -referenceIndex;
	continue;
      }
      
      if (currentNormal.dot(referenceNormal) < _inlierNormalAngularThreshold) {
	referenceIndex = -referenceIndex;
	continue;
      }

      float referenceCurvature = referenceStats[referenceIndex].curvature();
      float currentCurvature = currentStats[currentIndex].curvature();
      if (referenceCurvature < _flatCurvatureThreshold)
	referenceCurvature = _flatCurvatureThreshold;
      if (currentCurvature < _flatCurvatureThreshold)
	currentCurvature = _flatCurvatureThreshold;
      
      float logRatio = log(referenceCurvature + 1e-5) - log(currentCurvature + 1e-5);
      if (fabs(logRatio) > _inlierCurvatureRatioThreshold) {
	referenceIndex = -referenceIndex;
	continue;
      }
      
      correspondences[correspondenceIndex].referenceIndex = referenceIndex;
      correspondences[correspondenceIndex].currentIndex = currentIndex;
      
      correspondenceIndex++;
    }

    _numCorrespondences = correspondenceIndex;

    for (size_t i = _numCorrespondences; i < correspondences.size(); i++) {
      correspondences[i].referenceIndex = -1;
      correspondences[i].currentIndex = -1;
    }    
  }
}
