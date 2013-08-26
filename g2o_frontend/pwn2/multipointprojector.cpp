#include "multipointprojector.h"
#include "g2o_frontend/basemath/bm_se3.h"
#include "g2o_frontend/boss/serializable.h"

using namespace std;

namespace pwn {
  using namespace boss;

  void MultiPointProjector::ChildProjectorInfo::serialize(ObjectData& data, IdContext& /*context*/) {
    data.setPointer("projector",pointProjector);
    t2v(sensorOffset).toBOSS(data,"sensorOffset");
    data.setInt("width",width);
    data.setInt("height",height);
  }
  void MultiPointProjector::ChildProjectorInfo::deserialize(ObjectData& data, IdContext&/* context*/){
    data.bindPointer("projector",_tempProjector);
    Vector6f v;
    v.fromBOSS(data,"sensorOffset");
    sensorOffset=v2t(v);
    width=data.getInt("width");
    height=data.getInt("height");
    if(indexImage.rows() != width || indexImage.cols() != height)
      indexImage.resize(width, height);    
  }

  void MultiPointProjector::ChildProjectorInfo::deserializeComplete(){
    pointProjector=dynamic_cast<PointProjector*>(_tempProjector);
  }

  void MultiPointProjector::size(int &rows, int &cols) {
    // Compute total image size
    rows = 0;
    cols = 0;
    for(size_t i = 0; i < _pointProjectors.size(); i++) {
      if(_pointProjectors[i].width > rows)
	rows = _pointProjectors[i].width;
      cols += _pointProjectors[i].height;
    }
  }

  void MultiPointProjector::project(Eigen::MatrixXi &indexImage, 
				    Eigen::MatrixXf &depthImage, 
				    const PointVector &points) const {
    // Compute total image size
    int maxWidth = 0;
    int totalHeight = 0;
    for(size_t i = 0; i < _pointProjectors.size(); i++) {
      if(_pointProjectors[i].width > maxWidth)
	maxWidth = _pointProjectors[i].width;
      totalHeight += _pointProjectors[i].height;
    }
  
    // Resize the output images
    indexImage.resize(maxWidth, totalHeight);
    depthImage.resize(maxWidth, totalHeight);
    depthImage.fill(std::numeric_limits<float>::max());
    indexImage.fill(-1);
  
    int columnOffset = 0;
    for(size_t i = 0; i < _pointProjectors.size(); i++) {
      ChildProjectorInfo& childInfo = _pointProjectors[i];
      //const int currentWidth = childInfo.width;
      const int currentHeight = childInfo.height;
      PointProjector *currentPointProjector = childInfo.pointProjector;
      Eigen::MatrixXi& currentIndexImage = childInfo.indexImage;
      Eigen::MatrixXf& currentDepthImage = childInfo.depthImage;
      if(currentPointProjector != 0) {
	//currentPointProjector->setTransform(transform() * _pointProjectors[i].sensorOffset);
	currentPointProjector->project(currentIndexImage, currentDepthImage, points);      
      
	for (int c=0; c<currentIndexImage.cols(); c++) {
	  for (int r=0; r<currentIndexImage.rows(); r++){
	    int i = currentIndexImage(r,c);
	    indexImage(r,c+columnOffset)=i;
	    depthImage(r,c+columnOffset)=currentDepthImage(r,c);
	  
	  }
	}

	//indexImage.block(0, columnOffset, currentWidth, currentHeight) = currentIndexImage;
	//depthImage.block(0, columnOffset, currentWidth, currentHeight) = currentDepthImage;
      }
      columnOffset += currentHeight;
    }
  }

  void MultiPointProjector::unProject(PointVector &points,
				      Eigen::MatrixXi &indexImage, 
				      const Eigen::MatrixXf &depthImage) const {
    indexImage.resize(depthImage.rows(), depthImage.cols());
    indexImage.fill(-1);
    points.clear();
    PointVector currentPoints;
    int columnOffset = 0;
    for(size_t i = 0; i < _pointProjectors.size(); i++) {
      const int width = _pointProjectors[i].width;
      const int height = _pointProjectors[i].height;

      PointProjector *currentPointProjector = _pointProjectors[i].pointProjector;
      if(currentPointProjector != 0) {
	//currentPointProjector->setTransform(transform() * _pointProjectors[i].sensorOffset);
	_pointProjectors[i].depthImage = depthImage.block(0, columnOffset, width, height);
	currentPointProjector->unProject(currentPoints,
					 _pointProjectors[i].indexImage, 
					 _pointProjectors[i].depthImage);
      

	for(int r = 0; r < _pointProjectors[i].indexImage.rows(); r++) {
	  for(int c = 0; c < _pointProjectors[i].indexImage.cols(); c++) {
	    if(_pointProjectors[i].indexImage(r, c) != -1)
	      _pointProjectors[i].indexImage(r, c) += points.size();
	  }
	}

	indexImage.block(0, columnOffset, width, height) = _pointProjectors[i].indexImage;
	points.insert(points.end(), currentPoints.begin(), currentPoints.end());
      }

      columnOffset += height;
    }
  }

  void MultiPointProjector::unProject(PointVector &points,
				      Gaussian3fVector &gaussians,
				      Eigen::MatrixXi &indexImage,
				      const Eigen::MatrixXf &depthImage) const {
    indexImage.resize(depthImage.rows(), depthImage.cols());
    indexImage.fill(-1);
    points.clear();
    gaussians.clear();
    PointVector currentPoints;
    Gaussian3fVector currentGaussians;
    int columnOffset = 0;
    for(size_t i = 0; i < _pointProjectors.size(); i++) {
      const int width = _pointProjectors[i].width;
      const int height = _pointProjectors[i].height;

      PointProjector *currentPointProjector = _pointProjectors[i].pointProjector;
      if(currentPointProjector != 0) {
	//currentPointProjector->setTransform(transform() * _pointProjectors[i].sensorOffset);
	_pointProjectors[i].depthImage = depthImage.block(0, columnOffset, width, height);
	currentPointProjector->unProject(currentPoints,
					 currentGaussians,
					 _pointProjectors[i].indexImage, 
					 _pointProjectors[i].depthImage);

	for(int r = 0; r < _pointProjectors[i].indexImage.rows(); r++) {
	  for(int c = 0; c < _pointProjectors[i].indexImage.cols(); c++) {
	    if(_pointProjectors[i].indexImage(r, c) != -1)
	      _pointProjectors[i].indexImage(r, c) += points.size();
	  }
	}

	indexImage.block(0, columnOffset, width, height) = _pointProjectors[i].indexImage;
	points.insert(points.end(), currentPoints.begin(), currentPoints.end());
	gaussians.insert(gaussians.end(), currentGaussians.begin(), currentGaussians.end());
      }

      columnOffset += height;
    }
  }

  void MultiPointProjector::projectIntervals(Eigen::MatrixXi& intervalImage, 
					     const Eigen::MatrixXf& depthImage, 
					     const float worldRadius,
					     const bool blackBorders) const {
    intervalImage.resize(depthImage.rows(), depthImage.cols());
    Eigen::MatrixXi currentIntervalImage;
    int columnOffset = 0;
    for(size_t i = 0; i < _pointProjectors.size(); i++) {
      const int width = _pointProjectors[i].width;
      const int height = _pointProjectors[i].height;

      if(currentIntervalImage.rows() != width || currentIntervalImage.cols() != height)
	currentIntervalImage.resize(width, height);

      PointProjector *currentPointProjector = _pointProjectors[i].pointProjector;
      if(currentPointProjector != 0) {
	//currentPointProjector->setTransform(transform() * _pointProjectors[i].sensorOffset);
	_pointProjectors[i].depthImage = depthImage.block(0, columnOffset, width, height);
	currentPointProjector->projectIntervals(currentIntervalImage,
						_pointProjectors[i].depthImage,
						worldRadius,
						blackBorders);
	intervalImage.block(0, columnOffset, width, height) = currentIntervalImage;
      }
      columnOffset += height;
    }
  }

  bool MultiPointProjector::project(int &x, int &y, float &f, const Point &p) const {
    // Compute total image size
    int maxWidth = 0;
    int totalHeight = 0;
    for(size_t i = 0; i < _pointProjectors.size(); i++) {
      if(_pointProjectors[i].width > maxWidth)
	maxWidth = _pointProjectors[i].width;
      totalHeight += _pointProjectors[i].height;
    }
  
    int columnOffset = 0;
    int currentX = -1, currentY = -1;
    float currentF = 0.0f;
    for(size_t i = 0; i < _pointProjectors.size(); i++) {
      const int width = _pointProjectors[i].width;
      const int height = _pointProjectors[i].height;
    
      PointProjector *currentPointProjector = _pointProjectors[i].pointProjector;
      if(currentPointProjector != 0) {
	//currentPointProjector->setTransform(transform() * _pointProjectors[i].sensorOffset);
	if(currentPointProjector->project(currentX, currentY, currentF, p)) {
	  if(currentF < 0.0f || 
	     currentX < 0 || currentX >= width || 
	     currentY < 0 || currentY >= height) {
	    currentX = -1; 
	    currentY = -1;
	    currentF = 0.0f;
	  }
	  else {
	    currentY += columnOffset;
	    break;
	  }
	}
	else {
	  currentX = -1; 
	  currentY = -1;
	  currentF = 0.0f;
	}
      }
      columnOffset += height;
    }

    x = currentX;
    y = currentY;
    f = currentF;

    if(x == -1 && y == -1 && f < 0.0f)
      return false;
    return true;
  }

  void MultiPointProjector::setTransform(const Eigen::Isometry3f &transform_) {
    PointProjector::setTransform(transform_);
    for(size_t i = 0; i < _pointProjectors.size(); i++) {
      PointProjector *currentPointProjector = _pointProjectors[i].pointProjector;
      if (currentPointProjector) {
	currentPointProjector->setTransform(transform_ * _pointProjectors[i].sensorOffset);
      }
    }
  }

  void MultiPointProjector::clearProjectors() {
    for(size_t i = 0; i < _pointProjectors.size(); i++) {
      if(_pointProjectors[i].pointProjector != 0)
	delete(_pointProjectors[i].pointProjector);
      _pointProjectors[i].pointProjector = 0;
    }
    _pointProjectors.clear();
  }


  void MultiPointProjector::serialize(boss::ObjectData& data, boss::IdContext& context){
    PointProjector::serialize(data,context);
    ArrayData* arr=new ArrayData;
    for(size_t i=0; i<_pointProjectors.size(); i++){
      arr->add(&_pointProjectors[i]);
    }
    data.setField("childProjectors",arr);
  }
  
  void MultiPointProjector::deserialize(boss::ObjectData& data, boss::IdContext& context){
    PointProjector::deserialize(data,context);
    ArrayData* arr=dynamic_cast<ArrayData*>(data.getField("childProjectors"));
    clearProjectors();
    for(size_t i=0; i<arr->size(); i++){
      ChildProjectorInfo& pp=dynamic_cast<ChildProjectorInfo&>((*arr)[i]);
      _pointProjectors.push_back(pp);
    }
    delete arr;
  }

  BOSS_REGISTER_CLASS(MultiPointProjector);

}
