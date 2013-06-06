#include "gaussian3.h"
#include "pinholepointprojector.h"

#include <fstream>
#include <sstream>

using namespace Eigen;
using namespace std;

namespace pwn {

typedef Eigen::DiagonalMatrix<float, 3> Diagonal3f;

Gaussian3fVector::Gaussian3fVector(size_t s, const Gaussian3f &p) { 
  resize(s);
  std::fill(begin(), end(), p);
}

void Gaussian3fVector::toDepthImage(DepthImage &depthImage, 
				    const Matrix3f &cameraMatrix, const Isometry3f &cameraPose, 
				    float dmax) const {
  PinholePointProjector pointProjector;
  pointProjector.setTransform(cameraPose);
  pointProjector.setCameraMatrix(cameraMatrix);
  depthImage.fill(std::numeric_limits<float>::max());

  for(const_iterator it = begin(); it != end(); it++) {
    Point mean(it->mean());
    int r = -1, c = -1;
    float depth = 0.0f;
    pointProjector.project(r, c, depth, mean);
    if(depth < 0 || depth > dmax || 
	r < 0 || r >= depthImage.rows() || 
	c < 0 || c >= depthImage.cols())
      continue;
    float &d =depthImage(r, c);
    if(d > depth)
      d = depth;
  }
}

void Gaussian3fVector::fromDepthImage(const DepthImage &depthImage, 
				      const Matrix3f &cameraMatrix, 
				      float dmax, float baseline, float alpha) {
  PinholePointProjector pointProjector;
  pointProjector.setTransform(Eigen::Isometry3f::Identity());
  pointProjector.setCameraMatrix(cameraMatrix);
  clear();
  resize(depthImage.rows() * depthImage.cols(), Gaussian3f());
  const float *f = depthImage.data();
  int k = 0;
  float fB = (baseline * cameraMatrix(0, 0));
  Matrix3f inverseCameraMatrix = cameraMatrix.inverse();
  Matrix3f J;
  for(int c = 0; c < depthImage.cols(); c++) {
    for(int r = 0; r < depthImage.rows(); r++, f++) {
      if(*f >= dmax || *f <= 0)
	continue;
      Point mean;
      pointProjector.unProject(mean, r, c, *f);
      float z = mean.z();
      float zVariation = (alpha * z * z) / (fB + z * alpha);
      J <<       
	z, 0, (float)c,
	0, z, (float)r,
	0, 0, 1;
      J = inverseCameraMatrix * J;
      Diagonal3f imageCovariance(1.0f, 1.0f, zVariation);
      Matrix3f cov = J * imageCovariance * J.transpose();
      at(k) = Gaussian3f(mean.head<3>(), cov);
      k++;
    }
  }
  resize(k);
}


void Gaussian3fVector::toIndexImage(Eigen::MatrixXi &indexImage, DepthImage &depthImage, 
				    const Matrix3f &cameraMatrix, const Eigen::Isometry3f &cameraPose, 
				    float dmax) const {
  PinholePointProjector pointProjector;
  pointProjector.setTransform(cameraPose);
  pointProjector.setCameraMatrix(cameraMatrix);
  depthImage.resize(indexImage.rows(), indexImage.cols());
  depthImage.fill(std::numeric_limits<float>::max());
  indexImage.fill(-1);
  int k = 0;
  for(const_iterator it = begin(); it!=end(); k++, it++) {
    Point mean(it->mean());
    int r = -1, c = -1;
    float depth = 0.0f;
    pointProjector.project(r, c, depth, mean);
    if(depth < 0 || depth > dmax || 
	r < 0 || r >= depthImage.rows() || 
	c < 0 || c >= depthImage.cols())
      continue;
    float &d = depthImage(r, c);
    int &index = indexImage(r, c);
    if(d > depth) {
      d = depth;
      index = k;
    }
  }
}

void Gaussian3fVector::toPointAndNormalVector(PointVector &destPoints, NormalVector &destNormals, bool eraseNormals) const {
  destPoints.resize(size());
  destNormals.resize(size());
  for(size_t k = 0; k < size(); k++) {
    Point &point = destPoints[k];
    Normal &normal = destNormals[k];    
    point.head<3>() = at(k).mean();
    if(eraseNormals)
      normal.setZero();
  }
}

void Gaussian3fVector::fromPointVector(const PointVector &points,
				       const PointProjector &pointProjector,
				       const int rows, const int cols,
				       /*float dmax,*/ float baseline, float alpha) {
  const PinholePointProjector *pinholePointProjector = dynamic_cast<const PinholePointProjector*>(&pointProjector);
  if(!pinholePointProjector) {
    clear();
    return;
  }

  if(points.size() != size())
    resize(points.size());

  size_t k = 0;
  float fB = baseline * pinholePointProjector->cameraMatrix()(0, 0);
  Matrix3f J;
  for(size_t i = 0; i < points.size(); i++) {
    const Point &point = points[i];
    int r = -1, c = -1;
    float z = 0.0f;
    pinholePointProjector->project(r, c, z, point);
    if(/*z <= 0 || z >  dmax ||*/ 
	r < 0 || r >= rows || 
	c < 0 || c >= cols)
      continue;
    float zVariation = (alpha * z * z) / (fB + z * alpha);
    J <<       
      z, 0, (float)c,
      0, z, (float)r,
      0, 0, 1;
    J = pinholePointProjector->inverseCameraMatrix() * J;
    Diagonal3f imageCovariance(1.0f, 1.0f, zVariation);
    Matrix3f cov = J * imageCovariance * J.transpose();
    at(i) = Gaussian3f(point.head<3>(), cov);
    k++;
  }
  resize(k);
}

}
