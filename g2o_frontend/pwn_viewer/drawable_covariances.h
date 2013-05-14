#ifndef DRAWABLE_COVARIANCES
#define DRAWABLE_COVARIANCES

#include "../pwn2/pointstats.h"
#include "drawable.h"

class DrawableCovariances : public Drawable {
 public:
  DrawableCovariances();
  DrawableCovariances(Eigen::Isometry3f transformation_, GLParameter *parameter_, PointStatsVector *covariances_);
  
  virtual void setCovariances(PointStatsVector *covariances_) { _covariances = covariances_; }
  virtual bool setParameter(GLParameter *parameter_);
  
  virtual PointStatsVector* covariances() { return _covariances; }
  virtual GLParameter* parameter() { return _parameter; };
  
  virtual void draw();

 protected:
  PointStatsVector *_covariances;
};  

#endif
