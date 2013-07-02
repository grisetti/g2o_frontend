#include "pwn_data.h"

#include <iomanip>

#include "g2o/core/factory.h"

#ifdef WINDOWS
#include <windows.h>
#endif

#ifdef G2O_HAVE_OPENGL
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif // G2O_HAVE_OPENGL
#endif // __APPLE__

#include <iostream>

using namespace std;
using namespace g2o;
using namespace pwn;

PWNData::PWNData(Frame *frame_) {
  _filename = "";
  _frame = frame_;
  _originPose = Eigen::Isometry3f::Identity();
  _originPose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  _dataContainer = 0;
  _frameModified = _frame;
}

PWNData::~PWNData() {
  if(_frame)		
    delete _frame;
}

//! read the data from a stream
bool PWNData::read(std::istream &is) {
  // Read filename
  is >> _filename;
  
  // Update frame
  _frame = 0;
  //update();

  return true;
}

//! write the data to a stream
bool PWNData::write(std::ostream &os) const {
  // Write filename
  os <<  _filename << " ";

  // Write frame
  writeOut();

  return true;
}

void PWNData::writeOut() const {
  if(_frameModified && _frame) {
    _frame->save(_filename.c_str(), 1, true, _originPose);
    _frameModified = false;
  }
}

void PWNData::update() {
  if(!_frame) {
    _frame = new Frame();
    _frame->load(_originPose, _filename.c_str());
    _frameModified = false;
  }
}

void PWNData::release() {
  if(_frame) {
    delete _frame;
    _frame = 0;
  }
}

bool PWNDataDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters *params_) {
  if (!DrawAction::refreshPropertyPtrs(params_))
    return false;
  if(_previousParams) {
    _beamsDownsampling = _previousParams->makeProperty<IntProperty>(_typeName + "::BEAMS_DOWNSAMPLING", 10);
    _pointSize = _previousParams->makeProperty<FloatProperty>(_typeName + "::POINT_SIZE", 1.0f);
  } 
  else {
    _beamsDownsampling = 0;
    _pointSize = 0;
  }
  return true;
}

HyperGraphElementAction* PWNDataDrawAction::operator()(HyperGraph::HyperGraphElement *element, 
						       HyperGraphElementAction::Parameters *params_) {
  if(typeid(*element).name() != _typeName)
    return 0;
  
  refreshPropertyPtrs(params_);
  if(!_previousParams)
    return this;

  if(_show && !_show->value())
    return this;
  
  PWNData *that = static_cast<PWNData*>(element);
  if(!that->frame())
    return this;

  Eigen::Isometry3f originPose = that->originPose();
  originPose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  
  glPushMatrix();
  
  int step = 1;
  if(_beamsDownsampling )
    step = _beamsDownsampling->value();
  if(_pointSize)
    glPointSize(_pointSize->value());
  else 
    glPointSize(1);

  glColor4f(1.0f, 0.0f, 0.0f, 1.0f);

  glBegin(GL_POINTS);
  
  for(size_t i = 0; i < that->frame()->points().size(); i += step)  {
    Point point = that->frame()->points()[i];
    Normal normal = that->frame()->normals()[i];
    point = originPose * point;
    normal = originPose * normal;
    glNormal3f(-normal.x(), -normal.y(), -normal.z());
    glVertex3f(point.x(), point.y(), point.z());
  }
	
  glEnd();
  
  glPopMatrix();
  
  return this;
}

G2O_REGISTER_TYPE(PWN_DATA, PWNData);
G2O_REGISTER_ACTION(PWNDataDrawAction);

