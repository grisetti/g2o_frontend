#ifndef DRAWABLE_COVARIANCES
#define DRAWABLE_COVARIANCES

#include "../pwn2/pointstats.h"
#include "gl_parameter_covariances.h"
#include "drawable.h"

namespace pwn {

class DrawableCovariances : public Drawable {
 public:
  DrawableCovariances();
  DrawableCovariances(Eigen::Isometry3f transformation_, GLParameter *parameter_, PointStatsVector *covariances_);
  
  virtual GLParameter* parameter() { return _parameter; }
  virtual PointStatsVector* covariances() { return _covariances; }

  virtual bool setParameter(GLParameter *parameter_);
  virtual void setCovariances(PointStatsVector *covariances_) { _covariances = covariances_; }
  
  virtual void draw();

 protected:
  GLParameterCovariances *_parameter;
  PointStatsVector *_covariances;
};  

}

#endif
