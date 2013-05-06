#include "pointwithnormal.h"
#include "pixelmapper.h"
#include <iostream>
#include <fstream>
#include <sstream>

using namespace Eigen;
using namespace std;

PointWithNormalVector::PointWithNormalVector(size_t s, const PointWithNormal& p):
  std::vector<PointWithNormal, Eigen::aligned_allocator<PointWithNormal> >(s, p) {
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
      // check if the normals are defined and are compatible with the viewPoint
      // this does a normal check against the viewpoint
      if (point.normal().dot(point.point()-cameraPose.translation()) <=0){
	index = k;
      } 
    }
  }
}

bool PointWithNormalVector::save(ostream& os, int step, bool binary) const {
  os << "POINTWITHNORMALVECTOR " << size()/step << " " << binary << endl; 
  for (size_t i = 0; i<size(); i+=step){
    const PointWithNormal& point = at(i);
    if (! binary) {
      os << "POINTWITHNORMAL ";
      for (int k=0; k<6; k++)
	os << point(k) << " ";
      os << endl;
    } else {
      os.write((const char*) &point,sizeof(PointWithNormal));
    }
  }
  return os.good();
}

bool PointWithNormalVector::load(istream & is){
  clear();
  char buf[1024];
  is.getline(buf, 1024);
  istringstream ls(buf);
  string tag;
  size_t numPoints;
  bool binary;
  ls >> tag;
  if (tag!="POINTWITHNORMALVECTOR")
    return false;
  ls >> numPoints >> binary;
  resize(numPoints);
  cerr << "reading " << numPoints << " points, binary :" << binary << endl;
  size_t k=0;
  while (k<size() && is.good()) {
    PointWithNormal& point=at(k);
    if (!binary) {
      is.getline(buf, 1024);
      istringstream ls(buf);
      string s;
      ls >> s;
      if (s!="POINTWITHNORMAL")
	continue;
      for (int i=0; i<6 && ls; i++) {
	ls >> point(i);
      }
    } else {
      is.read((char*) &point,sizeof(PointWithNormal));
    }
    k++;
  }
  return is.good();
}

bool PointWithNormalVector::save(const char* filename, int step, bool binary) const{
  ofstream os(filename);
  if (! os)
    return false;
  return save(os,step,binary);
}

bool PointWithNormalVector::load(const char* filename){
  ifstream is(filename);
  if (! is)
    return false;
  return load(is);
}

PointWithNormalVector operator*(Eigen::Isometry3f t, const PointWithNormalVector& points){
  PointWithNormalVector ret(points.size());
  for (size_t i = 0 ; i<points.size(); i++){
    ret[i]=t*points[i];
  }
  return ret;
}
