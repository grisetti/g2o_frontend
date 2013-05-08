#ifndef DRAWABLE_CORRESPONDENCES
#define DRAWABLE_CORRESPONDENCES

#include "../pwn/homogeneousvector4f.h"
#include "../pwn/correspondencegenerator.h"
#include "drawable.h"

class DrawableCorrespondences : public Drawable {
 public:
  DrawableCorrespondences();
  DrawableCorrespondences(Eigen::Isometry3f transformation_, GLParameter *parameter_,  int numCorrespondences_, HomogeneousPoint3fVector *referencePoints_, HomogeneousPoint3fVector *currentPoints_, CorrespondenceVector *correspondences_);
  
  void setReferencePointsTransformation(Eigen::Isometry3f referencePointsTransformation_) { _referencePointsTransformation = referencePointsTransformation_; }
  void setNumCorrespondences(int numCorrespondences_) { _numCorrespondences = numCorrespondences_; }
  void setCorrespondences(CorrespondenceVector *correspondences_) { _correspondences = correspondences_; }
  void setReferencePoints(HomogeneousPoint3fVector *referencePoints_) { _referencePoints = referencePoints_; }
  void setCurrentPoints(HomogeneousPoint3fVector *currentPoints_) { _currentPoints = currentPoints_; }
  virtual bool setParameter(GLParameter *parameter_);

  Eigen::Isometry3f referencePointsTransformation() { return _referencePointsTransformation; }
  int numCorrespondances() { return _numCorrespondences; }
  CorrespondenceVector* correspondences() { return _correspondences; }
  HomogeneousPoint3fVector* referencePoints() { return _referencePoints; }
  HomogeneousPoint3fVector* currentPoints() { return _currentPoints; }
  virtual GLParameter* parameter() { return _parameter; };
  
  virtual void draw();
 
 protected:
  Eigen::Isometry3f _referencePointsTransformation;
  int _numCorrespondences;
  CorrespondenceVector *_correspondences;
  HomogeneousPoint3fVector *_referencePoints;
  HomogeneousPoint3fVector *_currentPoints;
 
};  

#endif
