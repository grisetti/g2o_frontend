#ifndef DRAWABLE_NORMALS
#define DRAWABLE_NORMALS

#include "../pwn/pwn_defs.h"
#include "drawable.h"

class DrawableNormals : public Drawable {
 public:
  DrawableNormals();
  DrawableNormals(Eigen::Isometry3f transformation_, GLParameter *parameter_, int step_, Vector6fVector *normals_);
  virtual void setNormals(Vector6fVector *normals_) { _normals = normals_; }
  virtual bool setParameter(GLParameter *parameter_);
  virtual const Vector6fVector* normals() { return _normals; }
  virtual GLParameter* parameter() { return _parameter; };
  virtual void draw();

 protected:
  const Vector6fVector *_normals;
};  

#endif
