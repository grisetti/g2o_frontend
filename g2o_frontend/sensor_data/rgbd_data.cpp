/*
 * rgbd_data.cpp
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#include "rgbd_data.h"
#include <fstream>
#include <iomanip>
#include "g2o/stuff/macros.h"
#include "g2o/core/factory.h"
#include <Eigen/Dense>

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


RGBDData::RGBDData(Sensor* sensor_, cv::Mat* intensityImage_, cv::Mat* depthImage_)
{
  _paramIndex = -1;
  _baseFilename = "";
  _rgbdCameraSensor = (SensorRGBDCamera*)sensor_; 
  _intensityImage = intensityImage_;
  _depthImage = depthImage_;
  _dataContainer = 0;
  _intensityImageModified = _intensityImage;
  _depthImageModified = _depthImage;
}

RGBDData::~RGBDData(){
  if(_intensityImage)
    delete _intensityImage;
  if(_depthImage)		
    delete _depthImage;
}

//! read the data from a stream
bool RGBDData::read(std::istream& is) 
{
  is >> _paramIndex >> _baseFilename;
  double ts;
  is >> ts;
  setTimeStamp(ts);
  _intensityImage = 0;
  _depthImage = 0;
  update();
  return true;
}

//! write the data to a stream
bool RGBDData::write(std::ostream& os) const 
{
  os << paramIndex() << " ";
  os << " " <<  _baseFilename << " ";
  string hn = "hostname";
  os << FIXED(" " << _timeStamp << " " << hn << " " << _timeStamp);
  writeOut();
  return true;
}

void RGBDData::writeOut() const
{
  if (_intensityImageModified && _intensityImage) {
    string intensityName=_baseFilename+ "_intensity.pgm";
    cv::imwrite(intensityName.c_str(), *_intensityImage);
    _intensityImageModified = false;
    cerr << "I" << endl;
  }

  if (_depthImageModified && _depthImage) {
    string depthName=_baseFilename+ "_depth.pgm";
    cv::imwrite(depthName.c_str(), *_depthImage);
    _depthImageModified = false;
    cerr << "D" << endl;
  }
}

void RGBDData::update()
{
  if (!_intensityImage) 
  {
    _intensityImage = new cv::Mat();
    *_intensityImage = cv::imread((_baseFilename + "_intensity.pgm") .c_str(), -1);
    _intensityImageModified = false;
  }
  if (!_depthImage) {
    _depthImage = new cv::Mat();
    *_depthImage = cv::imread((_baseFilename + "_depth.pgm") .c_str(), -1);
    _depthImageModified = false;
  }
}

void RGBDData::setSensor(Sensor* rgbdCameraSensor_)
{
  _rgbdCameraSensor = dynamic_cast<SensorRGBDCamera*>(rgbdCameraSensor_) ;
}

void RGBDData::release()
{
  if (_intensityImage) 
  {
    delete _intensityImage;
    _intensityImage = 0;
  }
  if (_depthImage) 
  {
    delete _depthImage;
    _depthImage = 0;
  }
}

bool RGBDDataDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_)
{
  if (!DrawAction::refreshPropertyPtrs(params_))
    return false;
  if (_previousParams)
  {
    _beamsDownsampling = _previousParams->makeProperty<IntProperty>(_typeName + "::BEAMS_DOWNSAMPLING", 10);
    _pointSize = _previousParams->makeProperty<FloatProperty>(_typeName + "::POINT_SIZE", .05f);
  } 
  else 
  {
    _beamsDownsampling = 0;
    _pointSize = 0;
  }
  return true;
}

HyperGraphElementAction* RGBDDataDrawAction::operator()(HyperGraph::HyperGraphElement* element, 
						     	HyperGraphElementAction::Parameters* params_)
{
  if(typeid(*element).name()!=_typeName)
    return 0;
  
  refreshPropertyPtrs(params_);
  if (!_previousParams)
  {
    return this;
  }

  if (_show && !_show->value())
    return this;

  glPushMatrix();
  int step = 1;
  if(_beamsDownsampling )
    step = _beamsDownsampling->value();
  if(_pointSize)
    glPointSize(_pointSize->value());
  else 
    glPointSize(1);
  
  RGBDData* that = static_cast<RGBDData*>(element);
  const unsigned short* dptr = reinterpret_cast<unsigned short*>(that->depthImage()->data);
  const unsigned char* dptrIntensity = reinterpret_cast<unsigned char*>(that->intensityImage()->data);
	
  glBegin(GL_POINTS);
  
  g2o::HyperGraph::DataContainer* container = that->dataContainer();
  
  g2o::OptimizableGraph::Vertex* v= dynamic_cast<g2o::OptimizableGraph::Vertex*>(container);
  
  OptimizableGraph* g = v->graph();
  
  g2o::Parameter* p = g->parameters().getParameter(that->paramIndex());
  
  g2o::ParameterCamera* param = dynamic_cast<g2o::ParameterCamera*> (p);
  
  Eigen::Matrix3d K = param->Kcam();

  static const double fx = K(0, 0);
  static const double fy = K(1, 1);
  static const double center_x = K(0, 2);
  static const double center_y = K(1, 2);

  double unit_scaling = 0.001f;
  float constant_x = unit_scaling / fx;
  float constant_y = unit_scaling / fy;
  
  for(int i = 0; i < that->depthImage()->rows; i++)  {
    for(int j = 0; j < that->depthImage()->cols; j+=step) {
    	unsigned short d = *dptr;
	unsigned int color = (unsigned int)*dptrIntensity;
    	if(d != 0) {
	  // Computing the Cartesian coordinates of the current pixel
	  float x = (j - center_x) * d * constant_x;
	  float y = (i - center_y) * d * constant_y;
	  float z = ((float)d) * unit_scaling;
	  Eigen::Vector3d point(x, y, z);
	  Eigen::Isometry3d offset = param->offset();
	  Vector7d off = g2o::internal::toVectorQT(offset);
	  Eigen::Quaternion<double> q(off[6], off[3], off[4], off[5]);
	  Eigen::Vector3d t(off[0], off[1], off[2]);
	  Eigen::Matrix3d R = q.toRotationMatrix();
	  point = R*point;				
	  point = point + t;
	  float vertexColor = color/255.0f;
	  glColor4f(vertexColor, vertexColor, vertexColor, 0.5f);
				glNormal3f(-point(0), -point(1), -point(2));
				glVertex3f(point(0), point(1), point(2));
	}
    	dptr = dptr + step;
	dptrIntensity = dptrIntensity + step;
    } 
  }
	
  glEnd();
  glPopMatrix();
  return this;
}

G2O_REGISTER_TYPE(RGBD_DATA, RGBDData);
G2O_REGISTER_ACTION(RGBDDataDrawAction);
