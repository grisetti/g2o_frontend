#ifndef DRAWABLE_NORMALS
#define DRAWABLE_NORMALS

#include "../pwn/homogeneousvector4f.h"
#include "drawable.h"

class DrawableNormals : public Drawable {
 public:
  DrawableNormals();
  DrawableNormals(const Eigen::Isometry3f& transformation_, GLParameter *parameter_, const HomogeneousPoint3fVector &points_, const HomogeneousNormal3fVector &normals_);

  virtual void setPoints(HomogeneousPoint3fVector &points_) { _points = points_; }
  virtual void setNormals(HomogeneousNormal3fVector &normals_) { _normals = normals_; }
  virtual bool setParameter(GLParameter *parameter_);

  virtual const HomogeneousPoint3fVector& points() { return _points; }
  virtual const HomogeneousNormal3fVector& normals() { return _normals; }
  virtual GLParameter* parameter() { return _parameter; };

  virtual void draw();

 protected:
  HomogeneousPoint3fVector _points;
  HomogeneousNormal3fVector _normals;
};  

#endif
