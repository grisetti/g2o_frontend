#include "pinholepointprojector.h"

PinholePointProjector::PinholePointProjector() : PointProjector() {
  _cameraMatrix << 
    1.0, 0.0, 0.5, 
    0.0, 1.0, 0.5,
    0.0, 0.0, 1;
  _updateMatrices();
}

PinholePointProjector::~PinholePointProjector() {}

void inline PinholePointProjector::setTransform(const Eigen::Isometry3f &transform_) {
  _transform = transform_;
  _updateMatrices();
}

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

inline bool PinholePointProjector::project(int &x, int &y, float &f, const HomogeneousPoint3f &p) const {
  return _project(x, y, f, p);
}

inline bool PinholePointProjector::unProject(HomogeneousPoint3f &p, const int x, const int y, const float d) const {
  return _unProject(p, x, y, d);
}

inline int PinholePointProjector::projectInterval(const int x, const int y, const float d, const float worldRadius) const {
  return _projectInterval(x, y, d, worldRadius);
}

void PinholePointProjector::project(Eigen::MatrixXi &indexImage,
				    Eigen::MatrixXf &depthImage, 
				    const HomogeneousPoint3fVector &points) const {
  depthImage.resize(indexImage.rows(), indexImage.cols());
  depthImage.fill(std::numeric_limits<float>::max());
  indexImage.fill(-1);
  const HomogeneousPoint3f *point = &points[0];
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
					     const float worldRadius) const {
  intervalImage.resize(depthImage.rows(), depthImage.cols());
  int cpix = 0;
  for (int c=0; c<depthImage.cols(); c++){
    const float *f = &depthImage(0,c);
    int *i = &intervalImage(0,c);
    for (int r=0; r<depthImage.rows(); r++, f++, i++){
      *i = _projectInterval(r, c, *f, worldRadius);
      cpix++;
    }
  }
}

void PinholePointProjector::unProject(HomogeneousPoint3fVector& points, 
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
