#ifndef DRAWABLE_NORMALS
#define DRAWABLE_NORMALS

#include "../pwn2/homogeneousvector4f.h"
#include "gl_parameter_normals.h"
#include "drawable.h"

namespace pwn {

class DrawableNormals : public Drawable {
 public:
  DrawableNormals();
  DrawableNormals(const Eigen::Isometry3f& transformation_, GLParameter *parameter_, PointVector *points_, NormalVector *normals_);
  virtual ~DrawableNormals() { glDeleteLists(_normalDrawList, 1); }

  virtual GLParameter* parameter() { return _parameter; };
  virtual PointVector* points() { return _points; }
  virtual NormalVector* normals() { return _normals; }
  inline GLuint normalDrawList() { return _normalDrawList; }

  virtual bool setParameter(GLParameter *parameter_);
  virtual void setPoints(PointVector *points_) { _points = points_; }
  virtual void setNormals(NormalVector *normals_) { _normals = normals_; }
 
  virtual void draw();
  void updateNormalDrawList();

 protected:
  GLParameterNormals *_parameter;
  PointVector *_points;
  NormalVector *_normals;
  GLuint _normalDrawList; 
};  

}

#endif
