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
	int referenceIndex = referenceIndexImage(r, c);
	int currentIndex = currentIndexImage(r, c);
	if (referenceIndex < 0 || currentIndex < 0)
	  continue;
	const HomogeneousNormal3f& currentNormal = currentNormals.at(currentIndex);      
	const HomogeneousPoint3f& _referenceNormal=referenceNormals.at(referenceIndex);
	if (_referenceNormal.squaredNorm()<1e-3 || currentNormal.squaredNorm()<1e-3)
	  continue;
	const HomogeneousPoint3f& currentPoint = currentPoints.at(currentIndex);
	const HomogeneousPoint3f& _referencePoint=referencePoints.at(referenceIndex);
	
	// remappings
	HomogeneousPoint3f referencePoint = T*_referencePoint;
	Eigen::Vector4f pointsDistance = currentPoint - referencePoint;
	//dtot += pointsDistance.squaredNorm() ;
	//ndtot += (referenceNormal-currentNormal).squaredNorm() ;
	if (pointsDistance.squaredNorm() > _squaredThreshold)
	  continue;      
	
	HomogeneousNormal3f referenceNormal = T*_referenceNormal;
	
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
