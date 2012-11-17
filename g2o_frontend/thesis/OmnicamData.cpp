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

OmnicamData::OmnicamData(){
	_paramIndex = -1;
	_baseFilename = "none";
	_cameraParams = 0;
	_image = 0;
	_sensorNumber = 0;
}

OmnicamData::OmnicamData(double timestamp_) : StampedData(timestamp_){
  _paramIndex = -1;
  _baseFilename = "none";
  _cameraParams = 0;
  _image = 0;
  _sensorNumber = 0;
}

OmnicamData::~OmnicamData(){
	if (_image){
		delete _image;
		_image = 0;
	}
}

//! read the data from a stream
bool OmnicamData::read(std::istream& is) {
	int r;
	is >> r;	
	return false;
}

//! write the data to a stream
bool OmnicamData::write(std::ostream& os) const {
	os << "OMNICAM_DATA " << _sensorNumber << " " << _baseFilename
	
	return true;
}

G2O_REGISTER_TYPE(OMNICAM_DATA, OmnicamData);
//G2O_REGISTER_ACTION(OMNICAMDataDrawAction);
