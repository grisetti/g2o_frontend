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

#include <iostream>

using namespace g2o;
using namespace std;
using namespace Eigen;

DepthImageData::DepthImageData() : g2o::OptimizableGraph::Data(){
  depthImage = 0;
}

bool DepthImageData::read(std::istream& is) {
  is >> paramIndex >> _filename;
  is >> ts_sec >> ts_usec;
  _filename = _filename + "_depth.pgm";
  update();
  return true;
}

bool DepthImageData::write(std::ostream& os) const {
  os << paramIndex << " " <<  _filename << " ";
  os << ts_sec << " " << ts_usec;
  return true;
}

void DepthImageData::update() {
  if(!depthImage) {    
    depthImage = new DepthImage();
    depthImage->load(&_filename[0]);
  }
}

DepthImageDataDrawAction::DepthImageDataDrawAction(): DrawAction(typeid(DepthImageData).name()){}

bool DepthImageDataDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
  if (!DrawAction::refreshPropertyPtrs(params_))
    return false;
  if(_previousParams) {
    _beamsDownsampling = _previousParams->makeProperty<IntProperty>(_typeName + "::BEAMS_DOWNSAMPLING", 10);
    _pointSize = _previousParams->makeProperty<FloatProperty>(_typeName + "::POINT_SIZE", 1.0f);
  } else {
    _beamsDownsampling = 0;
    _pointSize= 0;
  }
  return true;
}

HyperGraphElementAction* DepthImageDataDrawAction::operator()(HyperGraph::HyperGraphElement* element, 
							      HyperGraphElementAction::Parameters* params_){
  if (typeid(*element).name()!=_typeName)
    return 0;
  
  refreshPropertyPtrs(params_);
  if (! _previousParams){
    return this;
  }
  
  if (_show && !_show->value())
    return this;
  
  PixelMapper pm;
  DepthImageData* that = static_cast<DepthImageData*>(element);
  DepthImage *depthImage = that->depthImage;
  glPushMatrix();
  int step = 1;
  if (_beamsDownsampling )
    step = _beamsDownsampling->value();
  if (_pointSize) {
    glPointSize(_pointSize->value());
  }
  glBegin(GL_POINTS);
  glColor4f(1.f,0.f,0.f,1.0f);
  for (int i=0; i<depthImage->rows(); i+=step){
    for (int j=0; j<depthImage->cols(); j+=step){ 
      float depth = (*depthImage)(i, j);
      if(depth <= 0 || depth >= 20)
	continue;
      Vector3f p = pm.unprojectPixel(j, i, depth);
      glNormal3f(-p[0], -p[1], -p[2]);
      glVertex3f(p[0], p[1], p[2]);
    }
  }
  glEnd();
  glPopMatrix();
  return this;
}

G2O_REGISTER_TYPE(DEPTH_IMAGE_DATA, DepthImageData);
G2O_REGISTER_ACTION(DepthImageDataDrawAction);
