#include "feature_data.h"

#include "g2o/stuff/macros.h"
#include "g2o/core/factory.h"
#include "g2o/types/slam2d/types_slam2d.h"

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

bool BaseFeatureData::read(std::istream& is){
  int readAppDim, readPosDim;
  is >> readAppDim;
  int adim = appearanceMeasurementDimension();
  if (adim !=readAppDim)
    return false;
  double app[adim];
  for (int i=0; i< adim; i++)
    is >> app[i];
  setAppearanceMeasurement(app);
  is >> readPosDim;
  int pdim = positionMeasurementDimension();
  if (pdim !=readPosDim)
    return false;
  double pos[pdim];
  for (int i=0; i< pdim; i++)
    is >> pos[i];
  setPositionMeasurement(pos);
  double m[pdim*pdim];
  for (int r=0; r<pdim; r++)
    for (int c=r; c<pdim; c++){
      double v;
      is >> v;
      m[r*pdim+c] = m[c*pdim+r] = v;
    }
  setPositionMeasurementInformation(m);
  return true;
}

bool BaseFeatureData::write(std::ostream& os) const{
  int adim = appearanceMeasurementDimension();
  os << adim << " ";
  double app[adim];
  appearanceMeasurement(app);
  for (int i=0; i<adim; i++)
    os << app[i] << " ";
  int pdim = positionMeasurementDimension();
  os << pdim << " ";
  double pos[pdim];
  positionMeasurement(pos);
  for (int i=0; i< pdim; i++)
    os << pos[i] << " ";
  double m[pdim*pdim];
  positionMeasurementInformation(m);
  for (int r=0; r<pdim; r++)
    for (int c=r; c<pdim; c++){
      os << m[r*pdim+c]  << " ";
    }
  return true;
}

  
int FeaturePointXYData::positionMeasurementDimension() const {
  return 2;
}
void FeaturePointXYData::positionMeasurement(double* d) const{
  Map<Vector2d>p(d);
  p = _positionMeasurement;
}

void FeaturePointXYData::setPositionMeasurement(const double* d){
  Map<const Vector2d>p(d);
  _positionMeasurement = p;
}

void FeaturePointXYData::positionMeasurementInformation(double* d) const{
  Map<Matrix2d> m (d);
  m = _positionInformation;
}

void FeaturePointXYData::setPositionMeasurementInformation(const double* d){
  Map<const Matrix2d> m (d);
  _positionInformation = m;

}

int FeaturePointXYData::appearanceMeasurementDimension() const{
  return 0;
}

void FeaturePointXYData::appearanceMeasurement(double*) const {
}

void FeaturePointXYData::setAppearanceMeasurement(const double*) {
}

g2o::OptimizableGraph::Vertex* FeaturePointXYData::constructLandmarkVertex() const {
  return new VertexPointXY();
}
g2o::OptimizableGraph::Edge* FeaturePointXYData::constructEdge() const {
  return new EdgeSE2PointXY();
}

FeaturePointXYDataDrawAction::FeaturePointXYDataDrawAction(): DrawAction(typeid(FeaturePointXYData).name()){
}

bool FeaturePointXYDataDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
  if (!DrawAction::refreshPropertyPtrs(params_))
    return false;
  if (_previousParams){
    _pointSize = _previousParams->makeProperty<FloatProperty>(_typeName + "::POINT_SIZE", 1.f);
    _lineWidth = _previousParams->makeProperty<FloatProperty>(_typeName + "::LINE_WIDTH", .05f);
  } else {
    _pointSize= 0;
    _lineWidth = 0;
  }
  return true;
}

HyperGraphElementAction* FeaturePointXYDataDrawAction::operator()(HyperGraph::HyperGraphElement* element, 
							      HyperGraphElementAction::Parameters* params_){
  if (typeid(*element).name()!=_typeName)
    return 0;
  
  refreshPropertyPtrs(params_);
  if (! _previousParams){
    return this;
  }

  if (_show && !_show->value())
    return this;

  FeaturePointXYData* that = static_cast<FeaturePointXYData*>(element);
  if (_pointSize) {
    glPointSize(_pointSize->value());
  } else 
    glPointSize(1);

  glColor4f(1.f,0.f,0.f,0.5f);
  glPushMatrix();
  glBegin(GL_POINTS);
  glVertex3f(that->positionMeasurement().x(), that->positionMeasurement().y(), 0);
  glEnd();
  if (_lineWidth && _lineWidth->value()>0 ) {
    glColor4f(1.f,0.f,0.f,0.5f);
    glLineWidth(_lineWidth->value());
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(that->positionMeasurement().x(), that->positionMeasurement().y(), 0);
    glEnd();
  }
  
  glPopMatrix();
  return this;
}

G2O_REGISTER_TYPE(DATA_FEATURE_POINTXY, FeaturePointXYData);
G2O_REGISTER_ACTION(FeaturePointXYDataDrawAction);
