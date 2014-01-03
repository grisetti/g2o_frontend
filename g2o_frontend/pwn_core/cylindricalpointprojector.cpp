#include "cylindricalpointprojector.h"

using namespace std;

namespace pwn {
  
  CylindricalPointProjector::CylindricalPointProjector() : PointProjector() {
    _angularFov = M_PI/2; // 180 deg fov
    _angularResolution = 360.0/M_PI; ;//0.5 deg resolution
    _verticalCenter = 240;
    _verticalFocalLenght = 240; // 45 deg fov on the y
    _updateMatrices();
    _maxDistance = 6.0f;
  }

  void CylindricalPointProjector::_updateMatrices() {
    _iT =_transform.inverse();
    _iT.matrix().block<1, 4>(3, 0) << 0.0f, 0.0f, 0.0f, 1.0f;
    _inverseAngularResolution = 1.0f / _angularResolution;
    _angularCenter = _angularResolution * _angularFov;
    _inverseVerticalFocalLenght = 1.0f / _verticalFocalLenght;
  }

  void  CylindricalPointProjector::scale(float scalingFactor){
    _imageRows *= scalingFactor;
    _imageCols *= scalingFactor;
    _verticalCenter *= scalingFactor;
    _verticalFocalLenght /=scalingFactor;
    _angularResolution *= scalingFactor;
    _imageRows *= scalingFactor;
    _imageCols *= scalingFactor;
  }

  void CylindricalPointProjector::project(IntImage &indexImage,
					  DepthImage &depthImage, 
					  const PointVector &points)  {
    assert(_imageRows && _imageCols && "CylindricalPointProjector: _imageRows and _imageCols are zero");

    indexImage.create(_imageRows, _imageCols);
    depthImage.create(indexImage.rows, indexImage.cols);
    std::cerr << "Size: " << indexImage.rows << " ... " << indexImage.cols << endl; 
    depthImage.setTo(std::numeric_limits<float>::max());
    indexImage.setTo(-1);
    const Point *point = &points[0];
    for (size_t i = 0; i < points.size(); i++, point++) {
      int x, y;
      float d;
      if(!_project(x, y, d, *point) ||
	 d < _minDistance || d > _maxDistance ||
	 x < 0 || x >= indexImage.cols ||
	 y < 0 || y >= indexImage.rows)
	 continue;
      std::cerr << "proj: " << x << " --- " << y << " --- " << d << std::endl;
      float &otherDistance = depthImage(y, x);
      int &otherIndex = indexImage(y, x);
      if(!otherDistance || otherDistance > d) {
	otherDistance = d;
	otherIndex = i;
      }
    }
  }

    void CylindricalPointProjector::unProject(PointVector &points, 
					      IntImage &indexImage,
					      const DepthImage &depthImage) const {
      assert(depthImage.rows > 0 && depthImage.cols > 0 && "CylindricalPointProjector: Depth image has zero dimensions");

      points.resize(depthImage.rows * depthImage.cols);
      int count = 0;
      indexImage.create(depthImage.rows, depthImage.cols);
      Point *point = &points[0];
      for(int r = 0; r < depthImage.rows; r++) {
	const float *f = &depthImage(r, 0);
	int *i = &indexImage(r, 0);
	for(int c = 0; c < depthImage.cols; c++, f++, i++) {
	  if(!_unProject(*point, r, c, *f)) {
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

    void CylindricalPointProjector::unProject(PointVector &points, 
					      Gaussian3fVector &gaussians,
					      IntImage &indexImage,
					      const DepthImage &depthImage) const {
      assert(depthImage.rows > 0 && depthImage.cols > 0 && "CylindricalPointProjector: Depth image has zero dimensions");

      points.resize(depthImage.rows * depthImage.cols);
      gaussians.resize(depthImage.rows * depthImage.cols);
      indexImage.create(depthImage.rows, depthImage.cols);
      int count = 0;
      Point *point = &points[0];
      Gaussian3f *gaussian = &gaussians[0];
      for(int r = 0; r < depthImage.rows; r++) {
	const float *f = &depthImage(r, 0);
	int *i = &indexImage(r, 0);
	for(int c = 0; c < depthImage.cols; c++, f++, i++) {      
	  if(!_unProject(*point, r, c, *f)) {
	    *i = -1;
	    continue;
	  }
	  Eigen::Matrix3f cov = Eigen::Matrix3f::Identity();
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

    void CylindricalPointProjector::projectIntervals(IntImage &intervalImage, 
						     const DepthImage &depthImage, 
						     const float worldRadius) const {
      assert(depthImage.rows > 0 && depthImage.cols > 0 && "CylindricalPointProjector: Depth image has zero dimensions");

      intervalImage.create(depthImage.rows, depthImage.cols);
      for(int r = 0; r < depthImage.rows; r++){
	const float *f = &depthImage(r, 0);
	int *i = &intervalImage(r, 0);
	for(int c = 0; c < depthImage.cols; c++, f++, i++) {
	  *i = _projectInterval(r, c, *f, worldRadius);
	}
      }
    }
  }
