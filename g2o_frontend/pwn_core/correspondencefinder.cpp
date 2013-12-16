#include "correspondencefinder.h"

#include <omp.h>

using namespace std;

namespace pwn {

  CorrespondenceFinder::CorrespondenceFinder() {
    _inlierDistanceThreshold = 0.5f;  
    _squaredThreshold = _inlierDistanceThreshold * _inlierDistanceThreshold;
    _inlierNormalAngularThreshold = cos(M_PI/6);
    _flatCurvatureThreshold = 0.02f;
    _inlierCurvatureRatioThreshold = 1.3f;
    _numCorrespondences = 0;
    _rows = 0;
    _cols = 0;
  }

  void CorrespondenceFinder::compute(const Cloud &referenceScene, const Cloud &currentScene, Eigen::Isometry3f T) {
    assert(_referenceIndexImage.rows > 0 && _referenceIndexImage.cols > 0 && "CorrespondenceFinder: _referenceIndexImage has zero size");
    assert(_currentIndexImage.rows > 0 && _currentIndexImage.cols > 0 && "CorrespondenceFinder: _currentIndexImage has zero size");
    assert(_referenceDepthImage.rows > 0 && _referenceDepthImage.cols > 0 && "CorrespondenceFinder: _referenceDepthImage has zero size");
    assert(_currentDepthImage.rows > 0 && _currentDepthImage.cols > 0 && "CorrespondenceFinder: _currentDepthImage has zero size");
    
    T.matrix().block<1, 4>(3, 0) << 0.0f, 0.0f, 0.0f, 1.0f;
    _numCorrespondences = 0;
    if((int)_correspondences.size() != _referenceIndexImage.rows * _referenceIndexImage.cols)
      _correspondences.resize(_referenceIndexImage.rows * _referenceIndexImage.cols);

    float minCurvatureRatio = 1.0f / _inlierCurvatureRatioThreshold;
    float maxCurvatureRatio = _inlierCurvatureRatioThreshold;

    // Construct an array of counters;
    int numThreads = omp_get_max_threads();
    int localCorrespondenceIndex[numThreads];
    int localOffset[numThreads];
    int rowsPerThread = _referenceIndexImage.rows / numThreads;
    int iterationsPerThread = (_referenceIndexImage.rows * _referenceIndexImage.cols) / numThreads;
    for(int i = 0; i < numThreads; i++) {
      localOffset[i] = i * iterationsPerThread;
      localCorrespondenceIndex[i] = localOffset[i];
    }

#pragma omp parallel 
    {
      int threadId = omp_get_thread_num();
      int rMin = threadId * rowsPerThread;
      int rMax = rMin + rowsPerThread;
      if(rMax > _referenceIndexImage.rows)
	rMax = _referenceIndexImage.rows;

      int &correspondenceIndex = localCorrespondenceIndex[threadId];
      for(int r = rMin;  r < rMax; r++) {
	const int* referenceRowBase = &_referenceIndexImage(r, 0);
	const int* currentRowBase = &_currentIndexImage(r, 0);
	for(int c = 0; c < _referenceIndexImage.cols; c++) {
	  const int referenceIndex = *(referenceRowBase + c);
	  const int currentIndex = *(currentRowBase + c);
	  if (referenceIndex < 0 || currentIndex < 0) {
	    continue;
	  }
	
	  const Normal &currentNormal = currentScene.normals()[currentIndex];
	  const Normal &_referenceNormal = referenceScene.normals()[referenceIndex];
	  const Point &currentPoint = currentScene.points()[currentIndex];
	  const Point &_referencePoint = referenceScene.points()[referenceIndex];

	  if(currentNormal.squaredNorm() == 0.0f || _referenceNormal.squaredNorm() == 0.0f) {
	    continue;
	  }

	  // Remappings
	  Point referencePoint = T * _referencePoint;
	  Normal referenceNormal = T * _referenceNormal;
	
	  // This condition captures the angluar offset, and is moved to the end of the loop
	  if(currentNormal.dot(referenceNormal) < _inlierNormalAngularThreshold) {
	    continue;
	  }

	  Eigen::Vector4f pointsDistance = currentPoint - referencePoint;
	  // The condition below has moved to the increment, fill the pipeline, baby
	  if(pointsDistance.squaredNorm() > _squaredThreshold) {
	    continue;     	
          }
	  float referenceCurvature = referenceScene.stats()[referenceIndex].curvature();
	  float currentCurvature = currentScene.stats()[currentIndex].curvature();
	  if(referenceCurvature < _flatCurvatureThreshold)
	    referenceCurvature = _flatCurvatureThreshold;

	  if(currentCurvature < _flatCurvatureThreshold)
	    currentCurvature = _flatCurvatureThreshold;

	  // The condition below has moved to the increment, fill the pipeline, baby
	  float curvatureRatio = (referenceCurvature + 1e-5) / (currentCurvature + 1e-5);
	  if(curvatureRatio < minCurvatureRatio || curvatureRatio > maxCurvatureRatio) {
	    continue;
	  }

	  _correspondences[correspondenceIndex].referenceIndex = referenceIndex;
	  _correspondences[correspondenceIndex].currentIndex = currentIndex;
	  correspondenceIndex++;
	}
      }
    }

    // Assemble the solution
    int k = 0;
    for(int t = 0; t < numThreads; t++) {
      for (int i=localOffset[t]; i < localCorrespondenceIndex[t]; i++)
	_correspondences[k++] = _correspondences[i];
    }
    _numCorrespondences = k;

    for(size_t i = _numCorrespondences; i < _correspondences.size(); i++)
      _correspondences[i] = Correspondence();
  }

}
