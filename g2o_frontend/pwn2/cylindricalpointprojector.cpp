#include "cylindricalpointprojector.h"

using namespace std;

namespace pwn {
  
  CylindricalPointProjector::CylindricalPointProjector(int id, boss::IdContext* context) : PointProjector(id,context) {
    _angularFov = M_PI/2; // 180 deg fov
    _angularResolution = 360.0/M_PI; ;//0.5 deg resolution
    _verticalCenter = 240;
    _verticalFocalLenght = 240; // 45 deg fov on the y
    _updateMatrices();
    _maxDistance = 6.0;
  }

  CylindricalPointProjector::~CylindricalPointProjector() {}

  void CylindricalPointProjector::_updateMatrices() {
    _iT =_transform.inverse();
    _iT.matrix().block<1, 4>(3, 0) << 0, 0, 0, 1;
    _inverseAngularResolution = 1./_angularResolution;
    _angularCenter=_angularResolution*_angularFov;
    _inverseVerticalFocalLenght = 1./_verticalFocalLenght;
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


  inline bool CylindricalPointProjector::project(int &x, int &y, float &f, const Point &p) const {
    return _project(x, y, f, p);
  }

  inline bool CylindricalPointProjector::unProject(Point &p, const int x, const int y, const float d) const {
    return _unProject(p, x, y, d);
  }

  inline int CylindricalPointProjector::projectInterval(const int x, const int y, const float d, const float worldRadius) const {
    return _projectInterval(x, y, d, worldRadius);
  }

  void CylindricalPointProjector::project(IntImage &indexImage,
					  Eigen::MatrixXf &depthImage, 
					  const PointVector &points)  {

    if (!_imageRows || ! _imageCols)
      throw std::runtime_error("projector image not set");

    indexImage.resize(_imageRows, _imageCols);
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

  void CylindricalPointProjector::projectIntervals(IntImage& intervalImage, 
						   const Eigen::MatrixXf& depthImage, 
						   const float worldRadius,
						   const bool /*blackBorders*/) const {
    if (!_imageRows || ! _imageCols)
      throw std::runtime_error("projector image not set");

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

  void CylindricalPointProjector::unProject(PointVector& points, 
					    IntImage& indexImage,
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

  void CylindricalPointProjector::unProject(PointVector &points, 
					    Gaussian3fVector &gaussians,
					    IntImage &indexImage,
					    const Eigen::MatrixXf &depthImage) const {
    points.resize(depthImage.rows()*depthImage.cols());
    gaussians.resize(depthImage.rows()*depthImage.cols());
    indexImage.resize(depthImage.rows(), depthImage.cols());
    int count = 0;
    Point *point = &points[0];
    Gaussian3f *gaussian = &gaussians[0];
    int cpix = 0;
    for (int c = 0; c < depthImage.cols(); c++) {
      const float *f = &depthImage(0, c);
      int *i = &indexImage(0, c);
      for (int r=0; r<depthImage.rows(); r++, f++, i++){      
	if(!_unProject(*point, r, c, *f)) {
	  *i = -1;
	  continue;
	}
	//cerr << __PRETTY_FUNCTION__ << ": not yet fully implemented" << endl;
	Eigen::Matrix3f cov= Eigen::Matrix3f::Identity();
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

  void CylindricalPointProjector::serialize(boss::ObjectData& data, boss::IdContext& context){
    PointProjector::serialize(data,context);
    data.setFloat("angularFov",angularFov());
    data.setFloat("angularResolution",angularResolution());
    data.setFloat("fy",verticalFocalLenght());
    data.setFloat("cy", verticalCenter());
  }

  void CylindricalPointProjector::deserialize(boss::ObjectData& data, boss::IdContext& context){
    PointProjector::deserialize(data,context);
    setAngularFov(data.getFloat("angularFov"));
    setAngularResolution(data.getFloat("angularResolution"));
    setVerticalFocalLenght(data.getFloat("fy"));
    setVerticalCenter(data.getFloat("cy"));
  }

  BOSS_REGISTER_CLASS(CylindricalPointProjector);
}
