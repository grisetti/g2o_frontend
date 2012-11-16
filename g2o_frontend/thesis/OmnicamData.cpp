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
}

OmnicamData::OmnicamData(double timestamp_) : SensorData(timestamp_){
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
bool OmnicamData::read(std::istream& is) {
	int r;
	is >> r;	
	return false;
}

//! write the data to a stream
bool OmnicamData::write(std::ostream& os) const {
	int firstId = _nextId;
	// write the robot pose vertex
	os << "VERTEX_SE2 " << firstId << " " << _pose[0] << " " << _pose[1] << " " << _pose[2] << std::endl;
	
	// for every landmark seen from here
	for(unsigned int i=0; i<_observations.size(); i++){
		// write the landmark position
		os << "VERTEX_XY " << firstId+i+1;
		_observed_landmarks[i].write(os);
		os << std::endl;
		
		// write the constraints
		os << "EDGE_BEARING_SE2_XY " << firstId << " " << firstId+i+1 << " " << _observations[i] << " " << 200;
	}
	return false;
}

bool OmnicamData::writeOut() const
{
	return false;
}

// set the odometry
void OmnicamData::setPose(double x, double y, double theta){
	_pose[0] = x;
	_pose[1] = y;
	_pose[2] = theta;
}

// add a constraint
void OmnicamData::addObservation(double bearing, g2o::VertexPointXY landmark){
	_observations.push_back(bearing);
	_observed_landmarks.push_back(landmark);
}

G2O_REGISTER_TYPE(OMNICAM_DATA, OmnicamData);
//G2O_REGISTER_ACTION(OMNICAMDataDrawAction);
