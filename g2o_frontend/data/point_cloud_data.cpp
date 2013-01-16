#include "point_cloud_data.h"

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

bool PointCloudData::read(std::istream& is) {
  is >> paramIndex >> _cloudFilename;
  is >> ts_sec >> ts_usec;
  update();
  return true;
}

  //! write the data to a stream
bool PointCloudData::write(std::ostream& os) const {
  os << paramIndex << " " <<  _cloudFilename << " ";
  os << ts_sec << " " << ts_usec;
  return true;
}

void PointCloudData::update(){
  if (!cloudMsg) {
    std::cerr << "Ci sono: " << _cloudFilename << std::endl;
    sensor_msgs::PointCloud2* cloud_blob=new sensor_msgs::PointCloud2();
    pcl::io::loadPCDFile (_cloudFilename, *cloud_blob);
    cloudMsg=sensor_msgs::PointCloud2Ptr(cloud_blob);
    if (cloudMsg){
      cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg (*cloud_blob, *cloud);
      cerr << "read cloud with " << cloud->size() << " points" << endl;
    }
  }
}

void PointCloudData::release(){
  cloudMsg=sensor_msgs::PointCloud2Ptr( (sensor_msgs::PointCloud2*) 0);
  cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr( (pcl::PointCloud<pcl::PointXYZ>*) 0);
}

PointCloudDataDrawAction::PointCloudDataDrawAction(): DrawAction(typeid(PointCloudData).name()){
}

bool PointCloudDataDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
  if (!DrawAction::refreshPropertyPtrs(params_))
    return false;
  if (_previousParams){
    _beamsDownsampling = _previousParams->makeProperty<IntProperty>(_typeName + "::BEAMS_DOWNSAMPLING", 20);
    _pointSize = _previousParams->makeProperty<FloatProperty>(_typeName + "::POINT_SIZE", .05f);
  } else {
    _beamsDownsampling = 0;
    _pointSize= 0;
  }
  return true;
}

HyperGraphElementAction* PointCloudDataDrawAction::operator()(HyperGraph::HyperGraphElement* element, 
							      HyperGraphElementAction::Parameters* params_){
  if (typeid(*element).name()!=_typeName)
    return 0;
  
  refreshPropertyPtrs(params_);
  if (! _previousParams){
    return this;
  }

  if (_show && !_show->value())
    return this;

  PointCloudData* that = static_cast<PointCloudData*>(element);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = that->cloud;
  glPushMatrix();
  int step = 1;
  if (_beamsDownsampling )
    step = _beamsDownsampling->value();
  if (_pointSize) {
    glPointSize(_pointSize->value());
    }
  glBegin(GL_POINTS);
  glColor4f(1.f,0.f,0.f,0.5f);
  for (size_t i=0; i<cloud->size(); i+=step){
    glNormal3f(-cloud->points[i].x, -cloud->points[i].y, -cloud->points[i].z);
    glVertex3f(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
  }
  glEnd();
  glPopMatrix();
  return this;
}

G2O_REGISTER_TYPE(DATA_PCL, PointCloudData);
G2O_REGISTER_ACTION(PointCloudDataDrawAction);
