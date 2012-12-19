#ifndef DRAWABLE_COVARIANCES
#define DRAWABLE_COVARIANCES

#include "../pwn/pwn_defs.h"
#include "drawable.h"

class DrawableCovariances : public Drawable {
 public:
  DrawableCovariances();
  DrawableCovariances(Eigen::Isometry3f transformation_, GLParameter *parameter_,int step_, CovarianceSVDVector *covariances_);
  virtual void setCovariances(CovarianceSVDVector *covariances_) { _covariances = covariances_; }
  virtual bool setParameter(GLParameter *parameter_);
  virtual const CovarianceSVDVector* covariances() { return _covariances; }
  virtual GLParameter* parameter() { return _parameter; };
  virtual void draw();

 protected:
  const CovarianceSVDVector *_covariances;
};  

#endif
