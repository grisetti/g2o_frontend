#include "depth_registered_data.h"
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
    cout << "leto imagine " << _baseFilename << "_depth.png" << endl;
    cout << "colone: " << _depthImage->cols << endl;
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
  
	unsigned char* iptr=reinterpret_cast<unsigned char*>(that->_intensityImage->data);
  unsigned short* dptr=reinterpret_cast<unsigned short*>(that->_depthImage->data);
  assert(that->_intensityImage->rows == that->_depthImage->rows && 
  			 that->_intensityImage->cols == that->_depthImage->cols);
  int w = that->_intensityImage->cols;
  int h = that->_intensityImage->rows;

  float f = 575; 
  register float constant = 1.0f/f;
 	int v = -h/2;
  int k = 0;
  
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
  glBegin(GL_POINTS);
  glColor3f(1.f,1.f,1.f);
  
  cout  << "w: " << w << " h:" << h << endl;
  glPushAttrib(GL_LIGHTING);
  glDisable(GL_LIGHTING);
  for(int i = 0; i < that->_depthImage->rows; i++)  {
    int u = -w/2;
    for(int j = 0; j < that->_depthImage->cols; j++) {
      unsigned short d = *dptr;
      if(d != 0) {
	  float z = d * 1e-3f;
	  float x = u * z * constant;
	  float y = v * z * constant;
	  //glNormal3f(-x, -y, -z);
	  glVertex3f(x, y, z);
	}
      iptr++;
      dptr++;
      k++;
    }
    cout << endl;
  }
  glPopAttrib();
  
  glEnd();
  glPopMatrix();
  return this;
}

G2O_REGISTER_TYPE(DATA_DEPTH_REGISTERED, DepthRegisteredData);
G2O_REGISTER_ACTION(DepthRegisteredDataDrawAction);
