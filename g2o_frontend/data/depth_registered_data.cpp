#include "depth_registered_data.h"
#include <fstream>
#include "g2o/stuff/macros.h"
#include "g2o/core/factory.h"
//#include "depth_traits.h"

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

DepthRegisteredData::DepthRegisteredData()
{
  _paramIndex = -1;
  _baseFilename = "none";
  _cameraParams = 0;
  _ts_sec = 0;
  _ts_usec = 0;
  _intensityImage = 0;
  _depthImage = 0;
}

DepthRegisteredData::~DepthRegisteredData(){}

bool DepthRegisteredData::read(std::istream& is) 
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
bool DepthRegisteredData::write(std::ostream& os) const 
{
  if(_cameraParams)
    os << _cameraParams->id();
  else
    os << -1;
  os << " " <<  _baseFilename << " ";
  os << _ts_sec << " " << _ts_usec;
  return true;
}

void DepthRegisteredData::update()
{
  if(!_intensityImage) 
  {
    _intensityImage = new cv::Mat();
    _depthImage = new cv::Mat();
    *_intensityImage = cv::imread((_baseFilename + "_intensity.png") .c_str(), -1);
    *_depthImage = cv::imread((_baseFilename + "_depth.png") .c_str(), -1);
    cout << "letto immagine " << _baseFilename << "_depth.png" << endl;
  }
}

void DepthRegisteredData::release()
{
  if(_intensityImage) 
  {
    delete _intensityImage;
    _intensityImage = 0;
  }
  if(_depthImage) 
  {
    delete _depthImage;
    _depthImage = 0;
  }
}

DepthRegisteredDataDrawAction::DepthRegisteredDataDrawAction(): DrawAction(typeid(DepthRegisteredData).name())
{
}

bool DepthRegisteredDataDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_)
{
  if(!DrawAction::refreshPropertyPtrs(params_))
    return false;
  if(_previousParams)
  {
    _beamsDownsampling = _previousParams->makeProperty<IntProperty>(_typeName + "::BEAMS_DOWNSAMPLING", 20);
    _pointSize = _previousParams->makeProperty<FloatProperty>(_typeName + "::POINT_SIZE", .05f);
  } 
  else 
  {
    _beamsDownsampling = 0;
    _pointSize= 0;
  }
  return true;
}

HyperGraphElementAction* DepthRegisteredDataDrawAction::operator()(HyperGraph::HyperGraphElement* element, 
							      HyperGraphElementAction::Parameters* params_)
{
	DepthRegisteredData* that = static_cast<DepthRegisteredData*>(element);

  if(typeid(*element).name()!=_typeName)
    return 0;
  
  refreshPropertyPtrs(params_);
  if(! _previousParams)
  {
    return this;
  }

  if(_show && !_show->value())
    return this;

  glPushMatrix();
  //int step = 1;
  if(_beamsDownsampling )
    //step = _beamsDownsampling->value();
  if(_pointSize)
    glPointSize(_pointSize->value());
  
  unsigned short* dptr=reinterpret_cast<unsigned short*>(that->_depthImage->data);
	
  
  glBegin(GL_POINTS);
  glColor4f(1.f,1.f,1.f,1.f);
  glPushAttrib(GL_LIGHTING);
  glDisable(GL_LIGHTING);
  static const double fx = 5.758157348632812e+02;
	static const double fy = 5.758157348632812e+02;
	static const double center_x = 3.145e+02;
	static const double center_y = 2.355e+02;
	double unit_scaling = 1.0f*0.001f;
  float constant_x = unit_scaling / fx;
  float constant_y = unit_scaling / fy;
  
  
  for(int i = 0; i < that->_depthImage->rows; i++)  {
    for(int j = 0; j < that->_depthImage->cols; j++) {
    	unsigned short d = *dptr;
    	if(d != 0)
      {
      	// Computing the Cartesian coordinates of the current pixel
      	float x = (j - center_x) * d * constant_x;
      	float y = (i - center_y) * d * constant_y;
      	float z = (float)d * 0.001f;
      	glNormal3f(-x, -y, -z);
    		glVertex3f(x, y, z);
    	}
    	dptr++;
    } 
	}
	glPopAttrib();
  glEnd();
  glPopMatrix();
  return this;
}

G2O_REGISTER_TYPE(DATA_DEPTH_REGISTERED, DepthRegisteredData);
G2O_REGISTER_ACTION(DepthRegisteredDataDrawAction);
