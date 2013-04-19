#include "pointprojector.h"
#include <iostream>

PointProjector::PointProjector(){
  _transform.setIdentity();
  _minDistance = 0.01;
  _maxDistance = 10.0f;
}

PointProjector::~PointProjector(){
}


const Eigen::Isometry3f& PointProjector::transform() const {
  return _transform;
}

void PointProjector::setTransform(const Eigen::Isometry3f& transform_) {
  _transform=transform_;
}

void PointProjector::project(Eigen::MatrixXi& indexImage, 
			     Eigen::MatrixXf& depthImage, 
			     const HomogeneousPoint3fVector& points) const {
  depthImage.resize(indexImage.rows(), indexImage.cols());
  depthImage.fill(std::numeric_limits<float>::max());
  indexImage.fill(-1);
  const HomogeneousPoint3f* point = &points[0];
  for (size_t i=0; i<points.size(); i++, point++){
    int x, y;
    float d;
    if (!project(x,y,d,*point)||
	x<0 || x>=indexImage.rows() ||
	y<0 || y>=indexImage.cols()  )
      continue;
    float& otherDistance=depthImage(x,y);
    int&   otherIndex=indexImage(x,y);
    if (otherDistance>d) {
      otherDistance = d;
      otherIndex    = i;
    }
  }
}

void PointProjector::projectIntervals(Eigen::MatrixXi& intervalImage, 
			      const Eigen::MatrixXf& depthImage, 
			      float worldRadius) const{
  intervalImage.resize(depthImage.rows(), depthImage.cols());
  int cpix=0;
  for (int c=0; c<depthImage.cols(); c++){
    const float* f = &depthImage(0,c);
    int* i =&intervalImage(0,c);
    for (int r=0; r<depthImage.rows(); r++, f++, i++){
      *i=projectInterval(r,c,*f, worldRadius);
      cpix++;
    }
  }
}

void PointProjector::unProject(HomogeneousPoint3fVector& points,
			       Eigen::MatrixXi& indexImage,
			       const Eigen::MatrixXf& depthImage) const {
  points.resize(depthImage.rows()*depthImage.cols());
  int count = 0;
  indexImage.resize(depthImage.rows(), depthImage.cols());
  HomogeneousPoint3f* point = &points[0];
  int cpix=0;
  for (int c=0; c<depthImage.cols(); c++){
    const float* f = &depthImage(0,c);
    int* i =&indexImage(0,c);
    for (int r=0; r<depthImage.rows(); r++, f++, i++){
      if (!unProject(*point, r,c,*f)){
	*i=-1;
	continue;
      }
      point++;
      cpix++;
      *i=count;
      count++;
    }
  }
  points.resize(count);
}

bool PointProjector::project(int&, int&, float&, const HomogeneousPoint3f&) const { return false; }

int PointProjector::projectInterval(int, int, float, float) const { return 0; }

bool PointProjector::unProject(HomogeneousPoint3f&, int, int, float) const { return false; }

