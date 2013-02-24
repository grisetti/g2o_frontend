#include "omnicam_data.h"
#include <fstream>
#include "g2o/stuff/macros.h"
#include "g2o/core/factory.h"

#ifdef WINDOWS
#include <windows.h>
#endif

#ifdef G2O_HAVE_OPENGL
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#endif

using namespace g2o;
using namespace std;


OmnicamData::OmnicamData(cv::Mat* image_, Sensor* sensor_){
  _baseFilename = "";
  _image = 0;
  _dataContainer = 0;
  _imageModified= false;
  setSensor(sensor_);
  _image = image_;  
}

OmnicamData::~OmnicamData(){
  release();
}

//! read the data from a stream
bool OmnicamData::read(std::istream& is) {
  int pi;
  is >> pi >> _baseFilename;
  setParamIndex(pi);
  is >> _timeStamp;
  // update();
  return is.good();
}

void OmnicamData::update(){
  if(_image != 0){
    delete _image;
  }
  _image = new cv::Mat();
  *_image = cv::imread(_baseFilename+"_omni.pgm");
  _imageModified = 0;
}

void OmnicamData::release(){
  if (_image) {
    _imageModified = 0;
    delete _image;
    _image =0;
  }
}

//! write the data to a stream
bool OmnicamData::write(std::ostream& os) const {
  os << paramIndex() << " " ;
  string hn = "hostname";
  os << FIXED(" " << _timeStamp << " " << hn << " " << _timeStamp);
  writeOut();
  return true;
}

//! saves the image on a file
void OmnicamData::writeOut() const
{
  if (_imageModified && _image) {
    string intensityName=_baseFilename+ "_intensity.pgm";
    cv::imwrite(intensityName.c_str(), *_image);
    _imageModified = false;
  }
}


void OmnicamData::setImage(cv::Mat* image_)
{
  _image = image_;
  _imageModified = true;
}

// OmnicamDataDrawAction methods are here, but they do nothing.
// If you want to do print something, fill them
bool OmnicamDataDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_)
{
  return DrawAction::refreshPropertyPtrs(params_);
}


HyperGraphElementAction* OmnicamDataDrawAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_)
{
  if(typeid(*element).name()!=_typeName)
    return 0;
  refreshPropertyPtrs(params_);
  return 0;
}

G2O_REGISTER_TYPE(OMNICAM_DATA, OmnicamData);
G2O_REGISTER_ACTION(OmnicamDataDrawAction);
