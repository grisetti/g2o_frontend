#ifndef DRAWABLE_POINTS
#define DRAWABLE_POINTS

#include "../pwn/pointwithnormal.h"
#include "drawable.h"

class DrawablePoints : public Drawable {
 public:
  DrawablePoints();
  DrawablePoints(Eigen::Isometry3f transformation_, GLParameter *parameter_, int step_, PointWithNormalVector *points_);
  virtual void setPoints(PointWithNormalVector *points_) { _points = points_; }
  virtual bool setParameter(GLParameter *parameter_);
  virtual const PointWithNormalVector* points() { return _points; }
  virtual GLParameter* parameter() { return _parameter; };
  virtual void draw();

 protected:
  const PointWithNormalVector *_points;
};  

#endif
