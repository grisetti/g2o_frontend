#include "pinholepointprojector.h"

namespace pwn {

PinholePointProjector::PinholePointProjector() : PointProjector() {
  _cameraMatrix << 
    1.0, 0.0, 0.5, 
    0.0, 1.0, 0.5,
    0.0, 0.0, 1;
  _baseline = 0.075f;
  _alpha = 0.1f;
  _updateMatrices();
}

PinholePointProjector::~PinholePointProjector() {}

void inline PinholePointProjector::setCameraMatrix(const Eigen::Matrix3f &cameraMatrix_) {
  _cameraMatrix = cameraMatrix_;
  _updateMatrices();
}

void PinholePointProjector::_updateMatrices() {
  Eigen::Isometry3f t =_transform.inverse();
  t.matrix().block<1, 4>(3, 0) << 0, 0, 0, 1;
  _iK = _cameraMatrix.inverse();
  _KR = _cameraMatrix * t.linear();
  _Kt = _cameraMatrix * t.translation();
  _iKR = _transform.linear() * _iK;
  _iKt = _transform.translation();
  _KRt.setIdentity();
  _iKRt.setIdentity();
  _KRt.block<3, 3>(0, 0) = _KR; 
  _KRt.block<3, 1>(0, 3) = _Kt;
  _iKRt.block<3, 3>(0, 0) = _iKR; 
  _iKRt.block<3, 1>(0, 3) = _iKt;
}

inline bool PinholePointProjector::project(int &x, int &y, float &f, const Point &p) const {
  return _project(x, y, f, p);
}

inline bool PinholePointProjector::unProject(Point &p, const int x, const int y, const float d) const {
  return _unProject(p, x, y, d);
}

inline int PinholePointProjector::projectInterval(const int x, const int y, const float d, const float worldRadius) const {
  return _projectInterval(x, y, d, worldRadius);
}

void PinholePointProjector::project(Eigen::MatrixXi &indexImage,
				    Eigen::MatrixXf &depthImage, 
				    const PointVector &points) const {
  depthImage.resize(indexImage.rows(), indexImage.cols());
  depthImage.fill(std::numeric_limits<float>::max());
  indexImage.fill(-1);
  const Point *point = &points[0];
  for (size_t i=0; i<points.size(); i++, point++){
    int x, y;
    float d;
    if (!_project(x, y, d, *point)||
	d<_minDistance || 
	d>_maxDistance ||
	x<0 || x>=indexImage.rows() ||
	y<0 || y>=indexImage.cols()  )
      continue;
    float &otherDistance = depthImage.coeffRef(x,y);
    int &otherIndex = indexImage.coeffRef(x,y);
    if (otherDistance>d) {
      otherDistance = d;
      otherIndex = i;
    }
  }
}

void PinholePointProjector::projectIntervals(Eigen::MatrixXi& intervalImage, 
					     const Eigen::MatrixXf& depthImage, 
					     const float worldRadius,
					     const bool blackBorders) const {
  intervalImage.resize(depthImage.rows(), depthImage.cols());
  int cpix = 0;
  for (int c=0; c<depthImage.cols(); c++){
    const float *f = &depthImage(0,c);
    int *i = &intervalImage(0,c);
    for (int r=0; r<depthImage.rows(); r++, f++, i++){
      *i = _projectInterval(r, c, *f, worldRadius);
      if(blackBorders &&
	 ((r < *i) || (c < *i) || (depthImage.rows() - r < *i) || (depthImage.cols() - c < *i)))
	*i = -1;
      cpix++;
    }
  }
}

void PinholePointProjector::unProject(PointVector& points, 
				      Eigen::MatrixXi& indexImage,
				      const Eigen::MatrixXf& depthImage) const {
  points.resize(depthImage.rows()*depthImage.cols());
  int count = 0;
  indexImage.resize(depthImage.rows(), depthImage.cols());
  Point* point = &points[0];
  int cpix=0;
  for (int c=0; c<depthImage.cols(); c++){
    const float* f = &depthImage(0,c);
    int* i =&indexImage(0,c);
    for (int r=0; r<depthImage.rows(); r++, f++, i++){
      if (!_unProject(*point, r,c,*f)){
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

void PinholePointProjector::unProject(PointVector &points, 
				      Gaussian3fVector &gaussians,
				      Eigen::MatrixXi &indexImage,
				      const Eigen::MatrixXf &depthImage) const {
  points.resize(depthImage.rows()*depthImage.cols());
  gaussians.resize(depthImage.rows()*depthImage.cols());
  indexImage.resize(depthImage.rows(), depthImage.cols());
  int count = 0;
  Point *point = &points[0];
  Gaussian3f *gaussian = &gaussians[0];
  int cpix = 0;
  float fB = _baseline * _cameraMatrix(0,0);
  Eigen::Matrix3f J;
  for (int c = 0; c < depthImage.cols(); c++) {
    const float *f = &depthImage(0, c);
    int *i = &indexImage(0, c);
    for (int r=0; r<depthImage.rows(); r++, f++, i++){      
      if(!_unProject(*point, r, c, *f)) {
	*i = -1;
	continue;
      }
      float z = *f;
      float zVariation = (_alpha * z * z) / (fB + z * _alpha);
       J <<       
	z, 0, (float)r,
	0, z, (float)c,
	0, 0, 1;
      J = _iK * J;
      Diagonal3f imageCovariance(1.0f, 1.0f, zVariation);
      Eigen::Matrix3f cov = J * imageCovariance * J.transpose();
      *gaussian = Gaussian3f(point->head<3>(), cov);
      gaussian++;
      point++;
      cpix++;
      *i = count;
      count++;
    }
  }
  points.resize(count);
  gaussians.resize(count);
}

}
