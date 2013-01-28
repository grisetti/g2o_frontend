#ifndef DRAWABLE_NORMALS
#define DRAWABLE_NORMALS

#include "../pwn/pointwithnormal.h"
#include "drawable.h"

class DrawableNormals : public Drawable {
 public:
  DrawableNormals();
  DrawableNormals(const Eigen::Isometry3f& transformation_, GLParameter *parameter_,  const PointWithNormalVector *normals_);
  virtual void setNormals(PointWithNormalVector *normals_) { _normals = normals_; }
  virtual bool setParameter(GLParameter *parameter_);
  virtual const PointWithNormalVector* normals() { return _normals; }
  virtual GLParameter* parameter() { return _parameter; };
  virtual void draw();

 protected:
  const PointWithNormalVector *_normals;
};  

#endif
