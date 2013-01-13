#ifndef DRAWABLE_CORRESPONDENCES
#define DRAWABLE_CORRESPONDENCES

#include "../pwn/pointwithnormalaligner.h"
#include "drawable.h"

class DrawableCorrespondences : public Drawable {
 public:
  DrawableCorrespondences();
  DrawableCorrespondences(Eigen::Isometry3f transformation_, GLParameter *parameter_, int step_, int numCorrespondences_, PointWithNormalAligner::CorrespondenceVector *correspondences_);
  
  void setNumCorrespondences(int numCorrespondences_) { _numCorrespondences = numCorrespondences_; }
  int numCorrespondances() { return _numCorrespondences; }

  void setCorrespondences(const PointWithNormalAligner::CorrespondenceVector *correspondences_) { _correspondences = correspondences_; }
  const PointWithNormalAligner::CorrespondenceVector* correspondences() { return _correspondences; }

  void setPoints1(const PointWithNormalVector *points1_) { _points1 = points1_; }
  const PointWithNormalVector* points1() { return _points1; }

  void setPoints2(const PointWithNormalVector *points2_) { _points2 = points2_; }
  const PointWithNormalVector* points2() { return _points2; }

  virtual bool setParameter(GLParameter *parameter_);
  virtual GLParameter* parameter() { return _parameter; };
  virtual void draw();
 
  Eigen::Isometry3f _points2Transform;

 protected:
  int _numCorrespondences;
  const PointWithNormalAligner::CorrespondenceVector *_correspondences;
  const PointWithNormalVector *_points1;
  const PointWithNormalVector *_points2;
 
};  

#endif
