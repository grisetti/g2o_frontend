#include "gaussian3.h"
#include "pixelmapper.h"
#include <iostream>
#include <fstream>
#include <sstream>
using namespace Eigen;
using namespace std;

  typedef Eigen::DiagonalMatrix<float,3> Diagonal3f;

  Gaussian3fVector::Gaussian3fVector(size_t s, const Gaussian3f& p):
    std::vector<Gaussian3f, Eigen::aligned_allocator<Gaussian3f> >(s,p){
  }

void Gaussian3fVector::toDepthImage(MatrixXf& depthImage, 
					 const Matrix3f& cameraMatrix, const Isometry3f& cameraPose, 
					 float dmax) const {
  PixelMapper pixelMapper;
  pixelMapper.setTransform(cameraPose);
  pixelMapper.setCameraMatrix(cameraMatrix);
  depthImage.fill(std::numeric_limits<float>::max());

  for (const_iterator it = begin(); it!=end(); it++){
    const Vector3f& mean = it->mean();
    Vector3f ip=pixelMapper.projectPoint(mean);
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

void Gaussian3fVector::fromDepthImage(const Eigen::MatrixXf& depthImage, 
				      const Matrix3f& cameraMatrix, 
				      float dmax, float baseline, float alpha){
  PixelMapper pixelMapper;
  pixelMapper.setCameraMatrix(cameraMatrix);
  clear();
  resize(depthImage.rows()*depthImage.cols(), Gaussian3f());
  const float* f = depthImage.data();
  int k = 0;
  float fB = (baseline * cameraMatrix(0, 0));
  Matrix3f inverseCameraMatrix = cameraMatrix.inverse();
  Matrix3f J;
  for (int c=0; c<depthImage.cols(); c++) {
    for (int r=0; r<depthImage.rows(); r++, f++) {
      if (*f>=dmax || *f <=0 )
	continue;
      Vector3f mean=pixelMapper.unprojectPixel(c,r,*f);
      float z = mean(2);
      float zVariation = (alpha*z*z)/(fB+z*alpha);
      J <<       
	z, 0, (float)c,
	0, z, (float)r,
	0, 0, 1;
      J=inverseCameraMatrix * J;
      Diagonal3f imageCovariance(1.0f, 1.0f, zVariation);
      Matrix3f cov = J*imageCovariance*J.transpose();
      at(k) = Gaussian3f(mean,cov);
      k++;
    }
  }
  resize(k);
}


void Gaussian3fVector::toIndexImage(Eigen::MatrixXi& indexImage, Eigen::MatrixXf& zBuffer, 
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
    const Vector3f& mean = it->mean();
    Vector3f ip=pixelMapper.projectPoint(mean);
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

void Gaussian3fVector::toPointWithNormalVector(PointWithNormalVector& dest) const {
  dest.resize(size());
  for (size_t k =0; k<size(); k++){
    PointWithNormal& p = dest[k];
    p.head<3>()=at(k).mean();
    p.tail<3>().setZero();
  }
}


// bool Gaussian3fVector::save(const char* filename, int step) const{
//   ofstream os(filename);
//   if (! os)
//     return false;
//   for (size_t i = 0; i<size(); i+=step){
//     const Gaussian3f& point = at(i);
//     os << "GAUSSIAN3 ";
//     os << point.mean().transpose() << " ";
//     for (int r=0; r<3; r++)
//       for (int c=r; c<3; c++)
// 	os << point.omega()(r,c) << " ";
//     os << endl;
//   }
//   os.flush();
//   os.close();
//   return true;
// }

// bool Gaussian3fVector::load(const char* filename){
//   clear();
//   ifstream is(filename);
//   if (! is)
//     return false;
//   char buf[1024];
//   while (is.good()) {
//     is.getline(buf, 1024);
//     istringstream ls(buf);
//     string s;
//     ls >> s;
//     if (s!="POINTWITHNORMAL")
//       continue;
//     Gaussian3f p;
//     for (int i=0; i<6 && ls; i++) {
//       ls >> p(i);
//     }
//     push_back(p);
//   }
//   return is.good();
// }

// Gaussian3fVector operator*(Eigen::Isometry3f t, const Gaussian3fVector& points){
//   Gaussian3fVector ret(points.size());
//   for (size_t i = 0 ; i<points.size(); i++){
//     ret[i]=t*points[i];
//   }
//   return ret;
// }
