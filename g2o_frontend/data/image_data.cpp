#include "image_data.h"

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

bool ImageData::read(std::istream& is) {
  is >> paramIndex >> _imageFilename;
  is >> ts_sec >> ts_usec;
  _image=0;
  update();
  return true;
}

  //! write the data to a stream
bool ImageData::write(std::ostream& os) const {
  os << paramIndex << " " <<  _imageFilename << " ";
  os << ts_sec << " " << ts_usec;
  return true;
}

void ImageData::update(){
  if (!_image) {
    _image = new cv::Mat();
    *_image = cv::imread(_imageFilename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
  }
}

void ImageData::release(){
  delete _image;
  _image = 0;
}

// PointCloudDataDrawAction::PointCloudDataDrawAction(): DrawAction(typeid(PointCloudData).name()){
// }

// bool PointCloudDataDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
//   if (!DrawAction::refreshPropertyPtrs(params_))
//     return false;
//   if (_previousParams){
//     _beamsDownsampling = _previousParams->makeProperty<IntProperty>(_typeName + "::BEAMS_DOWNSAMPLING", 20);
//     _pointSize = _previousParams->makeProperty<FloatProperty>(_typeName + "::POINT_SIZE", .05f);
//   } else {
//     _beamsDownsampling = 0;
//     _pointSize= 0;
//   }
//   return true;
// }

// HyperGraphElementAction* PointCloudDataDrawAction::operator()(HyperGraph::HyperGraphElement* element, 
// 							      HyperGraphElementAction::Parameters* params_){
//   if (typeid(*element).name()!=_typeName)
//     return 0;
  
//   refreshPropertyPtrs(params_);
//   if (! _previousParams){
//     return this;
//   }

//   if (_show && !_show->value())
//     return this;

//   PointCloudData* that = static_cast<PointCloudData*>(element);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = that->cloud;
//   glPushMatrix();
//   int step = 1;
//   if (_beamsDownsampling )
//     step = _beamsDownsampling->value();
//   if (_pointSize) {
//     glPointSize(_pointSize->value());
//     }
//   glBegin(GL_POINTS);
//   glColor4f(1.f,0.f,0.f,0.5f);
//   for (size_t i=0; i<cloud->size(); i+=step){
//     glNormal3f(-cloud->points[i].x, -cloud->points[i].y, -cloud->points[i].z);
//     glVertex3f(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
//   }
//   glEnd();
//   glPopMatrix();
//   return this;
// }

G2O_REGISTER_TYPE(DATA_IMAGE, ImageData);
//G2O_REGISTER_ACTION(PointCloudDataDrawAction);
