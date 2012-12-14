#ifndef PWN_QGL_VIEWER
#define PWN_QGL_VIEWER
#include "pwn_cloud.h"
#include <QGLViewer/qglviewer.h>

class PWNQGLViewer: public QGLViewer{
public:
  PWNQGLViewer(QWidget* parent=0, const QGLWidget* shareWidget=0, Qt::WFlags flags=0);
  virtual void init();
  virtual void draw();

  void setPoints(const Vector6fVector* points_) {_points = points_;}
  void setPoints2(const Vector6fVector* points_) {_points2 = points_;}
  const Vector6fVector* points() {return _points;}
  void setEllipsoids(const CovarianceSVDVector* ellipsoids_) {_ellipsoids = ellipsoids_;}
  const CovarianceSVDVector* ellipsoids() {return _ellipsoids;}
  
  inline float normalLength() const { return _normalLength;}
  inline void setNormalLength(float normalLength_) {_normalLength = normalLength_;}
  inline float pointSize() const { return _pointSize;}
  inline void setPointSize(float pointSize_) {_pointSize = pointSize_;}
  inline float ellipsoidScale() const { return _ellipsoidScale;}
  inline void setEllipsoidScale(float ellipsoidScale_) {_ellipsoidScale = ellipsoidScale_;}
  inline float ellipsoidCrop() const { return _ellipsoidCrop;}
  inline void setEllipsoidCrop(float ellipsoidCrop_) {_ellipsoidCrop = ellipsoidCrop_;}

protected:
  float _normalLength;
  float _pointSize;
  float _ellipsoidScale;
  float _ellipsoidCrop;

  const Vector6fVector *_points;
  const Vector6fVector *_points2;  
  const CovarianceSVDVector *_ellipsoids;
  GLuint _ellipsoidList;
};

#endif
