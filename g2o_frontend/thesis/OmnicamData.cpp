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
  _baseFilename = "image";
  _cameraParams = 0;
  _image = 0;
  _sensorNumber = 0;
  _acquisitionNumber = 0;
}

OmnicamData::OmnicamData(){
  this->init();
}

OmnicamData::OmnicamData(double timestamp_) : SensorData(timestamp_){
  this->init();
}

void OmnicamData::setImage(cv::Mat* image_)
{
  _image = image_;
}


OmnicamData::~OmnicamData(){
	if (_image){
		delete _image;
		_image = 0;
	}
}

void OmnicamData::setSensorNumber(int sensorNumber_)
{
  _sensorNumber = sensorNumber_;
}

void OmnicamData::setAcquisitionNumber(int acquisitionNumber_){
  _acquisitionNumber = acquisitionNumber_;
}

//! read the data from a stream
bool OmnicamData::read(std::istream& is) {
	int r;
	is >> r;	
	return false;
}

static std::string computeFileName(std::string baseFileName_, int acquisitionNumber_)
{
  std::string filename;
  filename = baseFileName_;
  filename.append("_");
  
  char num[8];
  sprintf(num, "%05d", acquisitionNumber_);
  filename.append(num);
  
  filename.append(".pgm");
  return filename;
}

//! write the data to a stream
bool OmnicamData::write(std::ostream& os) const {
	
  std::string filename = computeFileName(_baseFilename, _acquisitionNumber);
	os << "OMNICAM_DATA " << _sensorNumber << " " << filename << std::endl;
	return true;
}

//! saves the image on a file
bool OmnicamData::writeOut() const
{
  std::string filename = computeFileName(_baseFilename, _acquisitionNumber);
  // save the image
    cv::imwrite(filename.c_str(), *_image);
    std::cout << "image saved to " << filename << std::endl;
  return true;
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
