#ifndef DRAWABLE_COVARIANCES
#define DRAWABLE_COVARIANCES

#include "../pwn/homogeneouspoint3fstats.h"
#include "drawable.h"

class DrawableCovariances : public Drawable {
 public:
  DrawableCovariances();
  DrawableCovariances(Eigen::Isometry3f transformation_, GLParameter *parameter_, HomogeneousPoint3fStatsVector &covariances_);
  
  virtual void setCovariances(const HomogeneousPoint3fStatsVector &covariances_) { _covariances = covariances_; }
  virtual bool setParameter(GLParameter *parameter_);
  
  virtual const HomogeneousPoint3fStatsVector& covariances() { return _covariances; }
  virtual GLParameter* parameter() { return _parameter; };
  
  virtual void draw();

 protected:
  HomogeneousPoint3fStatsVector _covariances;
};  

#endif
