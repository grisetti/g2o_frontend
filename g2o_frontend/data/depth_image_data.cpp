#include "depth_image_data.h"

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

DepthImageData::DepthImageData(){
  _paramIndex=-1;
  _filename = "none";
  _depthImage = 0;
  _cameraParams = 0;
  _ts_sec=0;
  _ts_usec=0;
}

DepthImageData::~DepthImageData(){
  release();
}

bool DepthImageData::read(std::istream& is) {
  int _paramIndex;
  is >> _paramIndex >> _filename;
  is >> _ts_sec >> _ts_usec;
  release();
  update();
  return true;
}

  //! write the data to a stream
bool DepthImageData::write(std::ostream& os) const {
  if (_cameraParams)
    os << _cameraParams->id();
  else
    os << -1;
  os << " " <<  _filename << " ";
  os << _ts_sec << " " << _ts_usec;
  return true;
}

void DepthImageData::update(){
  if (!_depthImage) {
    _depthImage = new cv::Mat();
    cerr << "reading: " << _filename << endl;
    *_depthImage = cv::imread( (_filename) .c_str(), -1);
  }
}

void DepthImageData::release(){
  if (_depthImage) {
    delete _depthImage;
    _depthImage = 0;
  }
}

DepthImageDataDrawAction::DepthImageDataDrawAction(): DrawAction(typeid(DepthImageData).name())
{
}

bool DepthImageDataDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_)
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

HyperGraphElementAction* DepthImageDataDrawAction::operator()(HyperGraph::HyperGraphElement* element, 
							      HyperGraphElementAction::Parameters* params_)
{
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
  int step = 1;
  if(_beamsDownsampling )
    step = _beamsDownsampling->value();
  if(_pointSize)
    glPointSize(_pointSize->value());
  
  DepthImageData* that = static_cast<DepthImageData*>(element);
  unsigned short* rptr=reinterpret_cast<unsigned short*>(that->_depthImage->data);
	
  glColor4f(1.f, 0.f, 0.f, 0.5f);
  static const double fx = 5.25e+02;
	static const double fy = 5.25e+02;
	static const double center_x = 3.195e+02;
	static const double center_y = 2.395e+02;
	double unit_scaling = 0.001f;
  float constant_x = unit_scaling / fx;
  float constant_y = unit_scaling / fy;
  
  glBegin(GL_POINTS);
  for(int i = 0; i < that->_depthImage->rows; i+=step)  {
    unsigned short* cptr = rptr;
    rptr += that->_depthImage->cols*step;
    for(int j = 0; j < that->_depthImage->cols; j+=step) {
    	unsigned short d = *cptr;
	cptr += step;
    	if(d != 0) {
	  // Computing the Cartesian coordinates of the current pixel
	  float x = (j - center_x) * d * constant_x;
	  float y = (i - center_y) * d * constant_y;
	  float z = ((float)d) * unit_scaling;
	  glNormal3f(-x, -y, -z);
	  glVertex3f(x, y, z);
    	}
    } 
  }
	
  glEnd();
  glPopMatrix();
  return this;
}

G2O_REGISTER_TYPE(DEPTH_IMAGE_DATA, DepthImageData);
G2O_REGISTER_ACTION(DepthImageDataDrawAction);


//G2O_REGISTER_ACTION(PointCloudDataDrawAction);
