#include "correspondencegenerator.h"
#include <iostream>
#include<omp.h>

using namespace std;

void CorrespondenceGenerator::compute(CorrespondenceVector &correspondences,
				      const HomogeneousPoint3fVector &referencePoints, const HomogeneousPoint3fVector &currentPoints,
				      const HomogeneousNormal3fVector &referenceNormals, const HomogeneousNormal3fVector &currentNormals,
				      const Eigen::MatrixXi &referenceIndexImage, const Eigen::MatrixXi &currentIndexImage,
				      const HomogeneousPoint3fStatsVector &referenceStats, const HomogeneousPoint3fStatsVector &currentStats,
				      const Eigen::Isometry3f &T_) {
  Eigen::Isometry3f T = T_;
  T.matrix().block<1,4>(3,0) << 0,0,0,1;
  _numCorrespondences = 0;
  if((int)correspondences.size() != referenceIndexImage.rows()*referenceIndexImage.cols())
    correspondences.resize(referenceIndexImage.rows()*referenceIndexImage.cols());

  float minCurvatureRatio = 1./_inlierCurvatureRatioThreshold;
  float maxCurvatureRatio = _inlierCurvatureRatioThreshold;
  

  // construct an array of counters;
  int numThreads = omp_get_max_threads();
  
  int localCorrespondenceIndex[numThreads];
  int localOffset[numThreads];
  int columnsPerThread = referenceIndexImage.cols()/numThreads;
  int iterationsPerThread = (referenceIndexImage.rows()*referenceIndexImage.cols())/numThreads;
  for (int i=0; i<numThreads; i++){
    localOffset[i] = i*iterationsPerThread;
    localCorrespondenceIndex[i] = localOffset[i];
  }
#pragma omp parallel 
  {
    int threadId = omp_get_thread_num();
    int cMin = threadId * columnsPerThread;
    int cMax = cMin + columnsPerThread;
    if (cMax > referenceIndexImage.cols())
      cMax = referenceIndexImage.cols();
    int& correspondenceIndex = localCorrespondenceIndex[threadId];
    for (int c = cMin;  c < cMax; c++) {
      for (int r = 0; r < referenceIndexImage.rows(); r++) {
	const int referenceIndex = referenceIndexImage(r, c);
	const int currentIndex = currentIndexImage(r, c);
	if (referenceIndex < 0 || currentIndex < 0)
	  continue;

	const HomogeneousNormal3f& currentNormal = currentNormals.at(currentIndex);      
	const HomogeneousPoint3f& _referenceNormal=referenceNormals.at(referenceIndex);
	const HomogeneousPoint3f& currentPoint = currentPoints.at(currentIndex);
	const HomogeneousPoint3f& _referencePoint=referencePoints.at(referenceIndex);
	

	// remappings
	HomogeneousPoint3f referencePoint = T*_referencePoint;
	HomogeneousNormal3f referenceNormal = T*_referenceNormal;
	// this condition captures the angluar offset, and is moved to the end of the loop
	if (currentNormal.dot(referenceNormal) < _inlierNormalAngularThreshold) 
	  continue;

	Eigen::Vector4f pointsDistance = currentPoint - referencePoint;
	// the condition below has moved to the increment, fill the pipeline, baby
	if (pointsDistance.squaredNorm() > _squaredThreshold)
	  continue;      
	
	
	
	float referenceCurvature = referenceStats[referenceIndex].curvature();
	float currentCurvature = currentStats[currentIndex].curvature();
	if (referenceCurvature < _flatCurvatureThreshold)
	  referenceCurvature = _flatCurvatureThreshold;

	if (currentCurvature < _flatCurvatureThreshold)
	  currentCurvature = _flatCurvatureThreshold;

	float curvatureRatio = (referenceCurvature + 1e-5)/(currentCurvature + 1e-5);
	// the condition below has moved to the increment, fill the pipeline, baby
	if (curvatureRatio < minCurvatureRatio || curvatureRatio > maxCurvatureRatio)
	  continue;

	// bool increment = 
	//   (currentNormal.dot(referenceNormal) > _inlierNormalAngularThreshold) &&
	//   (pointsDistance.squaredNorm() < _squaredThreshold) &&
	//   (curvatureRatio > minCurvatureRatio) && 
	//   (curvatureRatio < maxCurvatureRatio);

	correspondences[correspondenceIndex].referenceIndex = referenceIndex;
	correspondences[correspondenceIndex].currentIndex = currentIndex;
	correspondenceIndex++;

      }
    }
  }
  // assemble the solution
  int k=0;
  for (int t=0; t<numThreads; t++){
    //cerr << "assembling from " << localOffset[t] << " until " << localCorrespondenceIndex[t] << endl;
    for (int i=localOffset[t]; i< localCorrespondenceIndex[t]; i++)
      correspondences[k++]=correspondences[i];
  }
  _numCorrespondences = k;

  //std::cerr << "dtot: " << dtot << " ndtot: " << ndtot << std::endl;
  for (size_t i = _numCorrespondences; i < correspondences.size(); i++)
    correspondences[i] = Correspondence();
 
}
