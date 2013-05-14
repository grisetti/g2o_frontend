#ifndef DRAWABLE_CORRESPONDENCES
#define DRAWABLE_CORRESPONDENCES

#include "../pwn2/correspondencefinder.h"
#include "drawable.h"

class DrawableCorrespondences : public Drawable {
 public:
  DrawableCorrespondences();
  DrawableCorrespondences(Eigen::Isometry3f transformation_, GLParameter *parameter_,  int numCorrespondences_, PointVector *referencePoints_, PointVector *currentPoints_, CorrespondenceVector *correspondences_);
  
  void setReferencePointsTransformation(Eigen::Isometry3f referencePointsTransformation_) { _referencePointsTransformation = referencePointsTransformation_; }
  void setNumCorrespondences(int numCorrespondences_) { _numCorrespondences = numCorrespondences_; }
  void setCorrespondences(CorrespondenceVector *correspondences_) { _correspondences = correspondences_; }
  void setReferencePoints(PointVector *referencePoints_) { _referencePoints = referencePoints_; }
  void setCurrentPoints(PointVector *currentPoints_) { _currentPoints = currentPoints_; }
  virtual bool setParameter(GLParameter *parameter_);

  Eigen::Isometry3f referencePointsTransformation() { return _referencePointsTransformation; }
  int numCorrespondances() { return _numCorrespondences; }
  CorrespondenceVector* correspondences() { return _correspondences; }
  PointVector* referencePoints() { return _referencePoints; }
  PointVector* currentPoints() { return _currentPoints; }
  virtual GLParameter* parameter() { return _parameter; };
  
  virtual void draw();
 
 protected:
  Eigen::Isometry3f _referencePointsTransformation;
  int _numCorrespondences;
  CorrespondenceVector *_correspondences;
  PointVector *_referencePoints;
  PointVector *_currentPoints;
 
};  

#endif
