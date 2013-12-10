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

  void PinholePointProjector::_updateMatrices() {
    Eigen::Isometry3f t =_transform.inverse();
    t.matrix().block<1, 4>(3, 0) << 0.0f, 0.0f, 0.0f, 1.0f;
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

  void PinholePointProjector::project(IntImage &indexImage,
				      DepthImage &depthImage, 
				      const PointVector &points) const {
    assert(_imageRows && _imageCols && "PinholePointProjector: _imageRows and _imageCols are zero");
    indexImage.create(_imageRows, _imageCols);
    depthImage.create(indexImage.rows, indexImage.cols);
    depthImage.setTo(cv::Scalar(0.0f));
    indexImage.setTo(cv::Scalar(-1));
    const Point *point = &points[0];
    for(size_t i = 0; i < points.size(); i++, point++) {
      int x, y;
      float d;
      if(!_project(x, y, d, *point) ||
	 d < _minDistance || d > _maxDistance ||
	 x < 0 || x >= indexImage.cols ||
	 y < 0 || y >= indexImage.rows)
	continue;
      float &otherDistance = depthImage(y, x);
      int &otherIndex = indexImage(y, x);
      if(!otherDistance || otherDistance > d) {
	otherDistance = d;
	otherIndex = i;
      }
    }
  }

  void PinholePointProjector::unProject(PointVector &points, 
					IntImage &indexImage,
					const DepthImage &depthImage) const {
    assert(depthImage.rows > 0 && depthImage.cols > 0 && "PinholePointProjector: Depth image has zero dimensions");
    points.resize(depthImage.rows * depthImage.cols);
    int count = 0;
    indexImage.create(depthImage.rows, depthImage.cols);
    Point *point = &points[0];
    for(int r = 0; r < depthImage.rows; r++) {
      const float *f = &depthImage(r, 0);
      int *i = &indexImage(r, 0);
      for(int c = 0; c < depthImage.cols; c++, f++, i++) {
	if(!_unProject(*point, c, r, *f)) {
	  *i = -1;
	  continue;
	}
	point++;
	*i = count;
	count++;
      }
    }

    points.resize(count);
  }

  void PinholePointProjector::unProject(PointVector &points, 
					Gaussian3fVector &gaussians,
					IntImage &indexImage,
					const DepthImage &depthImage) const {
    assert(depthImage.rows > 0 && depthImage.cols > 0 && "PinholePointProjector: Depth image has zero dimensions");
    points.resize(depthImage.rows * depthImage.cols);
    gaussians.resize(depthImage.rows * depthImage.cols);
    indexImage.create(depthImage.rows, depthImage.cols);
    int count = 0;
    Point *point = &points[0];
    Gaussian3f *gaussian = &gaussians[0];
    float fB = _baseline * _cameraMatrix(0, 0);
    Eigen::Matrix3f J;
    for(int r = 0; r < depthImage.rows; r++) {
      const float *f = &depthImage(r, 0);
      int *i = &indexImage(r, 0);
      for(int c = 0; c < depthImage.cols; c++, f++, i++) {      
	if(!_unProject(*point, c, r, *f)) {
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
	Diagonal3f imageCovariance(3.0f, 3.0f, zVariation);
	Eigen::Matrix3f cov = J * imageCovariance * J.transpose();
	*gaussian = Gaussian3f(point->head<3>(), cov);
	gaussian++;
	point++;
	*i = count;
	count++;
      }
    }

    points.resize(count);
    gaussians.resize(count);
  }

  void PinholePointProjector::projectIntervals(IntImage &intervalImage, 
					       const DepthImage &depthImage, 
					       const float worldRadius,
					       const bool blackBorders) const {
    assert(depthImage.rows > 0 && depthImage.cols > 0 && "PinholePointProjector: Depth image has zero dimensions");
    intervalImage.create(depthImage.rows, depthImage.cols);
    for(int r = 0; r < depthImage.rows; r++) {
      const float *f = &depthImage(r, 0);
      int *i = &intervalImage(r, 0);
      for(int c = 0; c < depthImage.cols; c++, f++, i++) {
	*i = _projectInterval(r, c, *f, worldRadius);
	if(blackBorders &&
	   ((r < *i) || (c < *i) || (depthImage.rows - r < *i) || (depthImage.cols - c < *i)))
	  *i = -1;
      }
    }
  }

  void  PinholePointProjector::scale(float scalingFactor){
    _cameraMatrix.block<2, 3>(0, 0) *= scalingFactor;
    _imageRows *= scalingFactor;
    _imageCols *= scalingFactor;
  }

}
