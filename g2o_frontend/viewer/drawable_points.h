#ifndef DRAWABLE_POINTS
#define DRAWABLE_POINTS

#include "../pwn/pwn_defs.h"
#include "drawable.h"

class DrawablePoints : public Drawable {
 public:
  DrawablePoints();
  DrawablePoints(Eigen::Isometry3f transformation_, GLParameter *parameter_, float step_, Vector6fVector *points_);
  virtual void setPoints(Vector6fVector *points_) { _points = points_; }
  virtual bool setParameter(GLParameter *parameter_);
  virtual const Vector6fVector* points() { return _points; }
  virtual GLParameter* parameter() { return _parameter; };
  virtual void draw();

 protected:
  const Vector6fVector *_points;
};  

#endif
