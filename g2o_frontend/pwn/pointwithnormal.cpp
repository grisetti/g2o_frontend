#include "pointwithnormal.h"
#include "pixelmapper.h"
#include <iostream>
#include <fstream>
#include <sstream>
using namespace Eigen;
using namespace std;

  PointWithNormalVector::PointWithNormalVector(size_t s, const PointWithNormal& p):
    std::vector<PointWithNormal, Eigen::aligned_allocator<PointWithNormal> >(s,p){
  }

void PointWithNormalVector::toDepthImage(MatrixXf& depthImage, 
					 const Matrix3f& cameraMatrix, const Isometry3f& cameraPose, 
					 float dmax) const {
  PixelMapper pixelMapper;
  pixelMapper.setTransform(cameraPose);
  pixelMapper.setCameraMatrix(cameraMatrix);
  depthImage.fill(std::numeric_limits<float>::max());

  for (const_iterator it = begin(); it!=end(); it++){
    const PointWithNormal& point = *it;
    Vector3f ip=pixelMapper.projectPoint(point.head<3>());
    Vector2i coords=pixelMapper.imageCoords(ip);
    if (ip.z()<0 || ip.z() > dmax || 
	coords.x()<0 || coords.x()>=depthImage.cols() || 
	coords.y()<0 || coords.y()>=depthImage.rows())
      continue;
    float& d =depthImage(coords.y(), coords.x());
    if (d>ip.z())
      d=ip.z();
  }
}

void PointWithNormalVector::fromDepthImage(const Eigen::MatrixXf& depthImage, 
					   const Matrix3f& cameraMatrix, const Eigen::Isometry3f& cameraPose, 
					   float dmax){
  PixelMapper pixelMapper;
  pixelMapper.setTransform(cameraPose);
  pixelMapper.setCameraMatrix(cameraMatrix);
  clear();
  resize(depthImage.rows()*depthImage.cols(), PointWithNormal());
  const float* f = depthImage.data();
  int k = 0;
  for (int c=0; c<depthImage.cols(); c++) {
    for (int r=0; r<depthImage.rows(); r++, f++) {
      if (*f>=dmax || *f <=0 )
	continue;
      at(k).setZero();
      at(k).head<3>() = pixelMapper.unprojectPixel(c,r,*f);
      k++;
    }
  }
  resize(k);
}


void PointWithNormalVector::toIndexImage(Eigen::MatrixXi& indexImage, Eigen::MatrixXf& zBuffer, 
					 const Matrix3f& cameraMatrix, const Eigen::Isometry3f& cameraPose, 
					 float dmax) const {
  PixelMapper pixelMapper;
  pixelMapper.setTransform(cameraPose);
  pixelMapper.setCameraMatrix(cameraMatrix);
  zBuffer.resize(indexImage.rows(), indexImage.cols());
  zBuffer.fill(std::numeric_limits<float>::max());
  indexImage.fill(-1);
  int k=0;
  for (const_iterator it = begin(); it!=end(); k++ ,it++){
    const PointWithNormal& point = *it;
    Vector3f ip=pixelMapper.projectPoint(point.head<3>());
    Vector2i coords=pixelMapper.imageCoords(ip);
    if (ip.z()<0 || ip.z() > dmax || 
	coords.x()<0 || coords.x()>=zBuffer.cols() || 
	coords.y()<0 || coords.y()>=zBuffer.rows())
      continue;
    float& d =zBuffer(coords.y(), coords.x());
    int& index =indexImage(coords.y(), coords.x());
    if (d>ip.z()){
      d=ip.z();
      index = k;
    }
  }
}

bool PointWithNormalVector::save(const char* filename, int step) const{
  ofstream os(filename);
  if (! os)
    return false;
  for (size_t i = 0; i<size(); i+=step){
    const PointWithNormal& point = at(i);
    os << "POINTWITHNORMAL ";
    for (int k=0; k<6; k++)
      os << point(k) << " ";
    os << endl;
  }
  os.flush();
  os.close();
  return true;
}

bool PointWithNormalVector::load(const char* filename){
  clear();
  ifstream is(filename);
  if (! is)
    return false;
  char buf[1024];
  while (is.good()) {
    is.getline(buf, 1024);
    istringstream ls(buf);
    string s;
    ls >> s;
    if (s!="POINTWITHNORMAL")
      continue;
    PointWithNormal p;
    for (int i=0; i<6 && ls; i++) {
      ls >> p(i);
    }
    push_back(p);
  }
  return is.good();
}

PointWithNormalVector operator*(Eigen::Isometry3f t, const PointWithNormalVector& points){
  PointWithNormalVector ret(points.size());
  for (size_t i = 0 ; i<points.size(); i++){
    ret[i]=t*points[i];
  }
  return ret;
}
