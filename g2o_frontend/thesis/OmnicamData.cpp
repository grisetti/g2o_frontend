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

Omnicam::Omnicam(double timestamp_) : StampedData(timestamp_)
{
  _paramIndex = -1;
  _baseFilename = "none";
  _cameraParams = 0;
  _image = 0;
}

OmnicamData::~OmnicamData(){
	if (_image){
		delete _image;
		_image = 0;
	}
}

//! read the data from a stream
bool OmnicamData::read(std::istream& is) 
{
  int _paramIndex;
  is >> _paramIndex >> _baseFilename;
  is >> _ts_sec >> _ts_usec;
  _intensityImage = 0;
  _depthImage = 0;
  update();
  return true;
}

//! write the data to a stream
bool OmnicamData::write(std::ostream& os) const 
{
  if (_cameraParams)
    os << _cameraParams->id();
  else
    os << -1;
  os << " " <<  _baseFilename << " ";
  os << _ts_sec << " " << _ts_usec;
  return true;
}

// set the odometry
void setPose(double x, duble y, double theta){
	_pose[0] = x;
	_pose[1] = y;
	_pose[2] = theta;
}

G2O_REGISTER_TYPE(OMNICAM_DATA, OmnicamData);
//G2O_REGISTER_ACTION(OMNICAMDataDrawAction);
