#ifndef DRAWABLE_POINTS
#define DRAWABLE_POINTS

#include "../pwn/pwn_defs.h"
#include "drawable.h"

class DrawablePoints : public Drawable {
 public:
  DrawablePoints();
  DrawablePoints(Vector6fVector *points_, GLParameter *parameter_);
  virtual void setPoints(Vector6fVector *points_) { _points = points_; }
  virtual bool setParameter(GLParameter *parameter_);
  virtual Vector6fVector* points() { return _points; }
  virtual GLParameter* parameter() { return _parameter; };
  virtual void draw();

 protected:
  Vector6fVector *_points;
};  

#endif
