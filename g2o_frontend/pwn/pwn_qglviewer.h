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
  const Vector6fVector* points() {return _points;}
  void setEllipsoids(const Matrix6fVector* ellipsoids_) {_ellipsoids = ellipsoids_;}
  const Matrix6fVector* ellipsoids() {return _ellipsoids;}
  
protected:
  float _normalLength;
  float  _pointSize;
  float _ellipsoidsScale;
  
  const Vector6fVector *_points;
  const Matrix6fVector *_ellipsoids;
};

#endif
