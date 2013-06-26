#include "multipointprojector.h"

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
  
  int columnOffset = 0;
  for(size_t i = 0; i < _pointProjectors.size(); i++) {
    const int width = _pointProjectors[i].width;
    const int height = _pointProjectors[i].height;
    
    PointProjector *currentPointProjector = _pointProjectors[i].pointProjector;
    if(currentPointProjector != 0) {
      currentPointProjector->setTransform(_pointProjectors[i].sensorOffset);
      currentPointProjector->project(_pointProjectors[i].indexImage, _pointProjectors[i].depthImage, points);      
      indexImage.block(0, columnOffset, width, height) = _pointProjectors[i].indexImage;
      depthImage.block(0, columnOffset, width, height) = _pointProjectors[i].depthImage;
    }
    columnOffset += height;
  }
}

void MultiPointProjector::unProject(PointVector &points,
				    Eigen::MatrixXi &indexImage, 
				    const Eigen::MatrixXf &depthImage) const {
  indexImage.resize(depthImage.rows(), depthImage.cols());
  indexImage.fill(-1);
  points.clear();
  PointVector currentPoints;
  int columnOffset = 0;
  for(size_t i = 0; i < _pointProjectors.size(); i++) {
    const int width = _pointProjectors[i].width;
    const int height = _pointProjectors[i].height;

    PointProjector *currentPointProjector = _pointProjectors[i].pointProjector;
    if(currentPointProjector != 0) {
      currentPointProjector->setTransform(_pointProjectors[i].sensorOffset);
      _pointProjectors[i].depthImage = depthImage.block(0, columnOffset, width, height);
      currentPointProjector->unProject(currentPoints,
				       _pointProjectors[i].indexImage, 
				       _pointProjectors[i].depthImage);
      

      for(int r = 0; r < _pointProjectors[i].indexImage.rows(); r++) {
	for(int c = 0; c < _pointProjectors[i].indexImage.cols(); c++) {
	  if(_pointProjectors[i].indexImage(r, c) != -1)
	    _pointProjectors[i].indexImage(r, c) += points.size();
	}
      }

      indexImage.block(0, columnOffset, width, height) = _pointProjectors[i].indexImage;
      points.insert(points.end(), currentPoints.begin(), currentPoints.end());
    }

    columnOffset += height;
  }
}

void MultiPointProjector::unProject(PointVector &points,
				    Gaussian3fVector &gaussians,
				    Eigen::MatrixXi &indexImage,
				    const Eigen::MatrixXf &depthImage) const {
  indexImage.resize(depthImage.rows(), depthImage.cols());
  indexImage.fill(-1);
  points.clear();
  gaussians.clear();
  PointVector currentPoints;
  Gaussian3fVector currentGaussians;
  int columnOffset = 0;
  for(size_t i = 0; i < _pointProjectors.size(); i++) {
    const int width = _pointProjectors[i].width;
    const int height = _pointProjectors[i].height;

    PointProjector *currentPointProjector = _pointProjectors[i].pointProjector;
    if(currentPointProjector != 0) {
      currentPointProjector->setTransform(_pointProjectors[i].sensorOffset);
      _pointProjectors[i].depthImage = depthImage.block(0, columnOffset, width, height);
      currentPointProjector->unProject(currentPoints,
				       currentGaussians,
				       _pointProjectors[i].indexImage, 
				       _pointProjectors[i].depthImage);

      for(int r = 0; r < _pointProjectors[i].indexImage.rows(); r++) {
	for(int c = 0; c < _pointProjectors[i].indexImage.cols(); c++) {
	  if(_pointProjectors[i].indexImage(r, c) != -1)
	    _pointProjectors[i].indexImage(r, c) += points.size();
	}
      }

      indexImage.block(0, columnOffset, width, height) = _pointProjectors[i].indexImage;
      points.insert(points.end(), currentPoints.begin(), currentPoints.end());
      gaussians.insert(gaussians.end(), currentGaussians.begin(), currentGaussians.end());
    }

    columnOffset += height;
  }
}

void MultiPointProjector::projectIntervals(Eigen::MatrixXi& intervalImage, 
					   const Eigen::MatrixXf& depthImage, 
					   const float worldRadius,
					   const bool blackBorders) const {
  intervalImage.resize(depthImage.rows(), depthImage.cols());
  Eigen::MatrixXi currentIntervalImage;
  int columnOffset = 0;
  for(size_t i = 0; i < _pointProjectors.size(); i++) {
    const int width = _pointProjectors[i].width;
    const int height = _pointProjectors[i].height;

    if(currentIntervalImage.rows() != width || currentIntervalImage.cols() != height)
      currentIntervalImage.resize(width, height);

    PointProjector *currentPointProjector = _pointProjectors[i].pointProjector;
    if(currentPointProjector != 0) {
      currentPointProjector->setTransform(_pointProjectors[i].sensorOffset);
      _pointProjectors[i].depthImage = depthImage.block(0, columnOffset, width, height);
      currentPointProjector->projectIntervals(currentIntervalImage,
					      _pointProjectors[i].depthImage,
					      worldRadius,
					      blackBorders);
      intervalImage.block(0, columnOffset, width, height) = currentIntervalImage;
    }
    columnOffset += height;
  }
}

}
