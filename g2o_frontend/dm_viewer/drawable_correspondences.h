#ifndef DRAWABLE_CORRESPONDENCES
#define DRAWABLE_CORRESPONDENCES

#include "../dm_optimization/dm_defs.h"
#include "drawable.h"

class DrawableCorrespondences : public Drawable {
 public:
  DrawableCorrespondences();
  DrawableCorrespondences(Eigen::Isometry3f transformation_, GLParameter *parameter_, int step_, CorrespondenceVector *correspondences_);
  virtual void setCorrespondences(CorrespondenceVector *correspondences_) { _correspondences = correspondences_; }
  virtual bool setParameter(GLParameter *parameter_);
  virtual const CorrespondenceVector* correspondances() { return _correspondences; }
  virtual GLParameter* parameter() { return _parameter; };
  virtual void draw();

 protected:
  const CorrespondenceVector *_correspondences;
};  

#endif
