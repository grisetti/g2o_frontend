#ifndef DRAWABLE_COVARIANCES
#define DRAWABLE_COVARIANCES

#include "../pwn/pointwithnormalstatsgenerator.h"
#include "drawable.h"

class DrawableCovariances : public Drawable {
 public:
  DrawableCovariances();
  DrawableCovariances(Eigen::Isometry3f transformation_, GLParameter *parameter_,int step_, PointWithNormalSVDVector *covariances_);
  virtual void setCovariances(PointWithNormalSVDVector *covariances_) { _covariances = covariances_; }
  virtual bool setParameter(GLParameter *parameter_);
  virtual const PointWithNormalSVDVector* covariances() { return _covariances; }
  virtual GLParameter* parameter() { return _parameter; };
  virtual void draw();

 protected:
  const PointWithNormalSVDVector *_covariances;
};  

#endif
