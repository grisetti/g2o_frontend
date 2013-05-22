#ifndef DRAWABLE_POINTS
#define DRAWABLE_POINTS

#include "../pwn2/homogeneousvector4f.h"
#include "drawable.h"
#include "gl_parameter_points.h"

using namespace pwn;

class DrawablePoints : public Drawable {
 public:
  DrawablePoints();
  DrawablePoints(const Eigen::Isometry3f& transformation_, GLParameter *parameter_, PointVector *points_, NormalVector *normals_, 
    std::vector<int> *traversabilityVector_ );
  DrawablePoints(const Eigen::Isometry3f& transformation_, GLParameter *parameter_, PointVector *points_, NormalVector *normals_);

  virtual void setPoints(PointVector *points_) { _points = points_; }
  virtual void setNormals(NormalVector *normals_) { _normals = normals_; }
  virtual bool setParameter(GLParameter *parameter_);
  
  virtual PointVector* points() { return _points; }
  virtual NormalVector* normals() { return _normals; }
  virtual GLParameter* parameter() { return _parameter; };
  
  virtual void draw();

 protected:
  GLParameterPoints* _parameter;
  PointVector *_points;
  NormalVector *_normals;
  std::vector<int> *_traversabilityVector;
};  

#endif
