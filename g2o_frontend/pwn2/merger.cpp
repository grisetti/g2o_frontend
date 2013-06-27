#include "merger.h"

using namespace std;

namespace pwn {

Merger::Merger() {
  _distanceThreshold = 0.1f;
  _normalThreshold = cosf(20 * M_PI / 180.0f);
  _maxPointDepth = 10.0f;
  _depthImageConverter = 0;
  _indexImage.resize(0, 0);
  _depthImage.resize(0, 0);
  _collapsedIndices.resize(0);
}

void Merger::merge(Frame *frame, Eigen::Isometry3f transform) {
  assert(_depthImageConverter _indexImage.rows() != 0 && _indexImage.cols());
  //PinholePointProjector *pointProjector = dynamic_cast<PinholePointProjector*>(_depthImageConverter->_projector);
  PointProjector *pointProjector = _depthImageConverter->_projector;
  assert(pointProjector);
  pointProjector->setTransform(transform);
  pointProjector->project(_indexImage, 
			  _depthImage, 
			  frame->points());

  // scan all the points, 
  // if they fall in a cell not with -1, 
  //   skip
  // if they fall in a cell with n>1, 
  //   if distance is incompatible,
  //      skip
  // if normals are incompatible
  //      skip
  // accumulate the point in the cell i
  // set the target accumulator to i;
  _collapsedIndices.resize(frame->points().size());
  std::fill(_collapsedIndices.begin(), _collapsedIndices.end(), -1);
    
  int killed = 0;
  int currentIndex = 0;
  for(size_t i = 0; i < frame->points().size(); currentIndex++ ,i++) {
    const Point currentPoint = frame->points()[i];
    const Normal currentNormal = frame->normals()[i];
    
    int r = -1, c = -1;
    float depth = 0.0f;
    pointProjector->project(r, c, depth, currentPoint);
    if(depth < 0 || depth > _maxPointDepth || 
       r < 0 || r >= _depthImage.rows() || 
       c < 0 || c >= _depthImage.cols()) {
      continue;
    }
    
    float &targetZ = _depthImage(r, c);
    int targetIndex = _indexImage(r, c);
    const Normal &targetNormal = frame->normals()[targetIndex];
    if(targetIndex < 0) {
      continue;
    }
    if(targetIndex == currentIndex) {
      _collapsedIndices[currentIndex] = currentIndex;
    } 
    else if(fabs(depth - targetZ) < _distanceThreshold && currentNormal.dot(targetNormal) > _normalThreshold) {
      Gaussian3f &targetGaussian = frame->gaussians()[targetIndex];
      Gaussian3f &currentGaussian = frame->gaussians()[currentIndex];
      targetGaussian.addInformation(currentGaussian);
      _collapsedIndices[currentIndex]=targetIndex;
      killed++;
    }
  }
  //std::cerr << "Killed: " << killed << std::endl;
  
  // scan the vector of covariances.
  // if the index is -1
  //    copy into k
  //    increment k 
  // if the index is the same,
  //    update the point with normal
  //    copy into k
  //    increment k

  int murdered = 0;
  int k = 0;
  
  for(size_t i = 0; i < _collapsedIndices.size(); i++) {
    int collapsedIndex = _collapsedIndices[i];
    if(collapsedIndex == (int)i) {
      frame->points()[i].head<3>() = frame->gaussians()[i].mean();
    }
    if(collapsedIndex < 0 || collapsedIndex == (int)i) {
      frame->points()[k] = frame->points()[i];
      frame->normals()[k] = frame->normals()[i];
      frame->stats()[k] = frame->stats()[i];
      frame->pointInformationMatrix()[k] = frame->pointInformationMatrix()[i];
      frame->normalInformationMatrix()[k] = frame->normalInformationMatrix()[i];
      frame->gaussians()[k] = frame->gaussians()[i];
      k++;
    } 
    else {
      murdered ++;
    }
  }
    
  //int originalSize = frame->points().size();
  // kill the leftover points
  frame->points().resize(k);
  frame->normals().resize(k);
  frame->stats().resize(k);
  frame->pointInformationMatrix().resize(k);
  frame->normalInformationMatrix().resize(k);
  //cerr << "murdered: " << murdered  << endl;
  //cerr << "resized: " << originalSize << "->" << k << endl;
    
  // recompute the normals
  //pointProjector->project(_indexImage, _depthImage, frame->points());
  //_depthImageConverter->compute(*frame, _depthImage, transform);
}

}
