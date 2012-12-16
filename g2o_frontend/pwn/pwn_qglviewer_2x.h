#ifndef PWN_QGL_VIEWER_2X
#define PWN_QGL_VIEWER_2X
#include "pwn_cloud.h"
#include <QGLViewer/qglviewer.h>
#include <iostream>

struct Corr{
  Corr (const Vector6f* p1_, const Vector6f* p2_): p1(p1_), p2(p2_){
  }
  const Vector6f* p1, *p2;
};

typedef std::vector<Corr> CorrVector;

class PWNQGLViewer2X: public QGLViewer{
public:
  PWNQGLViewer2X(QWidget* parent=0, const QGLWidget* shareWidget=0, Qt::WFlags flags=0);
  virtual void init();
  virtual void draw();

  void setPoints0(const Vector6fVector* points0_) { _points0 = points0_;}
  void setPoints1(const Vector6fVector* points1_) { _points1 = points1_;}
  const Vector6fVector* points0() {return _points0;}
  const Vector6fVector* points1() {return _points1;}
  void setEllipsoids0(const CovarianceSVDVector* ellipsoids0_) { _ellipsoids0 = ellipsoids0_;}
  void setEllipsoids1(const CovarianceSVDVector* ellipsoids1_) { _ellipsoids1 = ellipsoids1_;}
  const CovarianceSVDVector* ellipsoids0() {return _ellipsoids0;}
  const CovarianceSVDVector* ellipsoids1() {return _ellipsoids1;}
  
  inline float normalLength() const { return _normalLength;}
  inline void setNormalLength(float normalLength_) {_normalLength = normalLength_;}
  inline float pointSize() const { return _pointSize;}
  inline void setPointSize(float pointSize_) {_pointSize = pointSize_;}
  inline float ellipsoidScale() const { return _ellipsoidScale;}
  inline void setEllipsoidScale(float ellipsoidScale_) {_ellipsoidScale = ellipsoidScale_;}
  inline float ellipsoidCrop() const { return _ellipsoidCrop;}
  inline void setEllipsoidCrop(float ellipsoidCrop_) {_ellipsoidCrop = ellipsoidCrop_;}

  inline const CorrVector* corrVector() const {return _corrVector;}
  inline void setCorrVector(const CorrVector* c)  {_corrVector = c;}

protected:
  float _normalLength;
  float _pointSize;
  float _ellipsoidScale;
  float _ellipsoidCrop;

  const Vector6fVector *_points0;
  const Vector6fVector *_points1;  
  const CovarianceSVDVector *_ellipsoids0;
  const CovarianceSVDVector *_ellipsoids1;
  const CorrVector *_corrVector;

  GLuint _ellipsoidList;
  GLuint _pyramidList;
};

#endif
