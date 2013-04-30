#ifndef DRAWABLE_CORRESPONDENCES
#define DRAWABLE_CORRESPONDENCES

#include "../pwn/homogeneousvector4f.h"
#include "../pwn/correspondencegenerator.h"
#include "drawable.h"

class DrawableCorrespondences : public Drawable {
 public:
  DrawableCorrespondences();
  DrawableCorrespondences(Eigen::Isometry3f transformation_, GLParameter *parameter_,  int numCorrespondences_, 
			  HomogeneousPoint3fVector &referencePoints_, HomogeneousPoint3fVector &currentPoints_, 
			  CorrespondenceVector &correspondences_);
  
  void setNumCorrespondences(int numCorrespondences_) { _numCorrespondences = numCorrespondences_; }
  int numCorrespondances() { return _numCorrespondences; }

  void setCorrespondences(const CorrespondenceVector &correspondences_) { _correspondences = correspondences_; }
  const CorrespondenceVector& correspondences() { return _correspondences; }

  void setReferencePoints(const HomogeneousPoint3fVector &referencePoints_) { _referencePoints = referencePoints_; }
  const HomogeneousPoint3fVector& referencePoints() { return _referencePoints; }

  void setCurrentPoints(const HomogeneousPoint3fVector &currentPoints_) { _currentPoints = currentPoints_; }
  const HomogeneousPoint3fVector& currentPoints() { return _currentPoints; }

  virtual bool setParameter(GLParameter *parameter_);
  virtual GLParameter* parameter() { return _parameter; };
  
  virtual void draw();
 
 protected:
  Eigen::Isometry3f _transformation;
  int _numCorrespondences;
  CorrespondenceVector _correspondences;
  HomogeneousPoint3fVector _referencePoints;
  HomogeneousPoint3fVector _currentPoints;
 
};  

#endif
