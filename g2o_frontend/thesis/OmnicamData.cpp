#include "OmnicamData.h"
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

void OmnicamData::init(){
  _paramIndex = -1;
  _baseFilename = "none";
  _omnicamSensor = 0;
  _ts_sec = 0;
  _ts_usec = 0;
  _image = 0;
  _dataContainer = 0;
}

OmnicamData::OmnicamData(){
  this->init();
}

OmnicamData::OmnicamData(cv::Mat* image_){
  this->init();
  _image = image_;
}

OmnicamData::~OmnicamData(){
  if(_image)
    delete _image;
}

//! read the data from a stream
bool OmnicamData::read(std::istream& is) {
  is >> _paramIndex >> _baseFilename;
  is >> _ts_sec >> _ts_usec;
  if(_image != 0){
    delete _image;
  }
  _image = new cv::Mat();
  *_image = cv::imread(_baseFilename+"omni.pgm");
	return false;
}

//! write the data to a stream
bool OmnicamData::write(std::ostream& os) const {
  if(_omnicamSensor)
    os << _omnicamSensor->getParameter()->id();
  else
    os << -1;
  os << " " << _baseFilename << " ";
  os << _ts_sec << " " << _ts_usec;
  return true;
}

//! saves the image on a file
void OmnicamData::writeOut()
{
  int num = _omnicamSensor->getNum();
  _omnicamSensor->setNum(num+1);
  
  char name[8];
  sprintf(name, "%05d", num);
  _baseFilename = string(name);
  
  char buf[25];
  sprintf(buf, "omni_%05d.pgm", num);
  cv::imwrite(buf, *_image);
  cout << "Saved omnicam image #" << num << endl;
}

// OmnicamDataDrawAction methods are here, but they do nothing.
// If you want to do print something, fill them
bool OmnicamDataDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_)
{
  return DrawAction::refreshPropertyPtrs(params_);
}

void OmnicamData::setImage(cv::Mat* image_)
{
  _image = image_;
}

void OmnicamData::setSensor(Sensor* omniSensor_){
  _omnicamSensor = dynamic_cast<SensorOmnicam*>(omniSensor_);
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
