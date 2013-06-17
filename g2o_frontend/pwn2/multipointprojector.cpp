#include "multipointprojector.h"

#include "../basemath/bm_se3.h"
#include <cstdio>
using namespace std;

namespace pwn {

void MultiPointProjector::project(Eigen::MatrixXi &indexImage, 
				  Eigen::MatrixXf &depthImage, 
				  const PointVector &points) const {
  // Compute total image size
  int maxWidth = 0;
  int totalHeight = 0;
  for(size_t i = 0; i < _pointProjectors.size(); i++) {
    if(_pointProjectors[i].width > maxWidth)
      maxWidth = _pointProjectors[i].width;
    totalHeight += _pointProjectors[i].height;
  }
  
  // Resize the output images
  indexImage.resize(maxWidth, totalHeight);
  depthImage.resize(maxWidth, totalHeight);
  depthImage.fill(std::numeric_limits<float>::max());
  indexImage.fill(-1);
  
  Eigen::MatrixXi currentIndexImage;
  Eigen::MatrixXf currentDepthImage;
  int columnOffset = 0;
  for(size_t i = 0; i < _pointProjectors.size(); i++) {
    const int width = _pointProjectors[i].width;
    const int height = _pointProjectors[i].height;
    if(currentIndexImage.rows() != width || currentIndexImage.cols() != height)
      currentIndexImage.resize(width, height);
    
    PointProjector *currentPointProjector = _pointProjectors[i].pointProjector;
    if(currentPointProjector != 0) {
      currentPointProjector->setTransform(_pointProjectors[i].sensorOffset);
      currentPointProjector->project(currentIndexImage, currentDepthImage, points);      
      indexImage.block(0, columnOffset, width, height) = currentIndexImage;
      depthImage.block(0, columnOffset, width, height) = currentDepthImage;
    }
    columnOffset += height;
  }
}

void MultiPointProjector::unProject(PointVector &points,
				    Eigen::MatrixXi &indexImage, 
				    const Eigen::MatrixXf &depthImage) const {
  indexImage.resize(depthImage.rows(), depthImage.cols());
  indexImage.fill(-1);
  points.resize(0);
  PointVector currentPoints;
  Eigen::MatrixXi currentIndexImage;
  int rowOffset = 0;
  for(size_t i = 0; i < _pointProjectors.size(); i++) {
    const int width = _pointProjectors[i].width;
    const int height = _pointProjectors[i].height;

    if(currentIndexImage.rows() != width || currentIndexImage.cols() != height)
      currentIndexImage.resize(width, height);

    PointProjector *currentPointProjector = _pointProjectors[i].pointProjector;
    if(currentPointProjector != 0) {
      currentPointProjector->setTransform(_pointProjectors[i].sensorOffset);
      currentPointProjector->unProject(currentPoints,
				       currentIndexImage, 
				       depthImage.block(width, height, rowOffset + width, 0));
      indexImage.block(width, height, rowOffset + width, 0) = currentIndexImage;
      points.insert(points.end(), points.begin(), currentPoints.end());
    }
    rowOffset += width;
  }
}

void MultiPointProjector::unProject(PointVector &points,
				    Gaussian3fVector &gaussians,
				    Eigen::MatrixXi &indexImage,
				    const Eigen::MatrixXf &depthImage) const {
  indexImage.resize(depthImage.rows(), depthImage.cols());
  indexImage.fill(-1);
  points.resize(0);
  gaussians.resize(0);
  PointVector currentPoints;
  Gaussian3fVector currentGaussians;
  Eigen::MatrixXi currentIndexImage;
  int rowOffset = 0;
  for(size_t i = 0; i < _pointProjectors.size(); i++) {
    const int width = _pointProjectors[i].width;
    const int height = _pointProjectors[i].height;

    if(currentIndexImage.rows() != width || currentIndexImage.cols() != height)
      currentIndexImage.resize(width, height);

    PointProjector *currentPointProjector = _pointProjectors[i].pointProjector;
    if(currentPointProjector != 0) {
      currentPointProjector->setTransform(_pointProjectors[i].sensorOffset);
      currentPointProjector->unProject(currentPoints,
				       currentGaussians,
				       currentIndexImage, 
				       depthImage.block(width, height, rowOffset + width, 0));
      indexImage.block(width, height, rowOffset + width, 0) = currentIndexImage;
      points.insert(points.end(), points.begin(), currentPoints.end());
      gaussians.insert(gaussians.end(), gaussians.begin(), currentGaussians.end());
    }
    rowOffset += width;
  }
}

void MultiPointProjector::projectIntervals(Eigen::MatrixXi& intervalImage, 
					   const Eigen::MatrixXf& depthImage, 
					   const float worldRadius) const {
  intervalImage.resize(depthImage.rows(), depthImage.cols());
  Eigen::MatrixXi currentIntervalImage;
  int rowOffset = 0;
  for(size_t i = 0; i < _pointProjectors.size(); i++) {
    const int width = _pointProjectors[i].width;
    const int height = _pointProjectors[i].height;

    if(currentIntervalImage.rows() != width || currentIntervalImage.cols() != height)
      currentIntervalImage.resize(width, height);
    PointProjector *currentPointProjector = _pointProjectors[i].pointProjector;
    if(currentPointProjector != 0) {
      currentPointProjector->setTransform(_pointProjectors[i].sensorOffset);
      currentPointProjector->projectIntervals(currentIntervalImage,
					      depthImage.block(width, height, rowOffset + width, 0),
					      worldRadius);
      intervalImage.block(width, height, rowOffset + width, 0) = currentIntervalImage;
    }
    rowOffset += width;
  }
}

}
