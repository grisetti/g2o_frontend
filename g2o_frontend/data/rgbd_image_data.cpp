#include "rgbd_image_data.h"

#include "g2o/stuff/macros.h"
#include "g2o/core/factory.h"

#ifdef WINDOWS
#include <windows.h>
#endif


using namespace g2o;
using namespace std;

RGBDImageData::RGBDImageData(){
  _paramIndex=-1;
  _baseFilename = "none";
  _cameraParams = 0;
  _ts_sec=0;
  _ts_usec=0;
}

bool RGBDImageData::read(std::istream& is) {
  int _paramIndex;
  is >> _paramIndex >> _baseFilename;
  is >> _ts_sec >> _ts_usec;
  _intensityImage=0;
  _depthImage=0;
  update();
  return true;
}

  //! write the data to a stream
bool RGBDImageData::write(std::ostream& os) const {
  if (_cameraParams)
    os << _cameraParams->id();
  else
    os << -1;
  os << " " <<  _baseFilename << " ";
  os << _ts_sec << " " << _ts_usec;
  return true;
}

void RGBDImageData::update(){
  if (!_intensityImage) {
    _intensityImage = new cv::Mat();
    _depthImage = new cv::Mat();
    *_intensityImage = cv::imread( (_baseFilename+"_GRAY.pgm") .c_str(), -1);
    *_depthImage = cv::imread( (_baseFilename+"_DEPT.pgm") .c_str(), -1);
  }
}

void RGBDImageData::release(){
  if (_intensityImage) {
    delete _intensityImage;
    _intensityImage = 0;
  }
  if (_depthImage) {
    delete _depthImage;
    _depthImage = 0;
  }

}

G2O_REGISTER_TYPE(DATA_DEPTH_IMAGE, RGBDImageData);
//G2O_REGISTER_ACTION(PointCloudDataDrawAction);
