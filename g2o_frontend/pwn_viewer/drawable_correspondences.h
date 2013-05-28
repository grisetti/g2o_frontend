#ifndef DRAWABLE_CORRESPONDENCES
#define DRAWABLE_CORRESPONDENCES

#include "g2o_frontend/pwn2/correspondencefinder.h"
#include "drawable.h"
#include "gl_parameter_correspondences.h"

namespace pwn {

class DrawableCorrespondences : public Drawable {
 public:
  DrawableCorrespondences();
  DrawableCorrespondences(Eigen::Isometry3f transformation_, GLParameter *parameter_,  int numCorrespondences_, PointVector *referencePoints_, 
			  PointVector *currentPoints_, CorrespondenceVector *correspondences_);
  virtual ~DrawableCorrespondences() { glDeleteLists(_correspondenceDrawList, 1); }
  
  virtual GLParameter* parameter() { return _parameter; };
  Eigen::Isometry3f referencePointsTransformation() { return _referencePointsTransformation; }
  int numCorrespondances() { return _numCorrespondences; }
  CorrespondenceVector* correspondences() { return _correspondences; }
  PointVector* referencePoints() { return _referencePoints; }
  PointVector* currentPoints() { return _currentPoints; }
  inline GLuint correspondenceDrawList() { return _correspondenceDrawList; }

  virtual bool setParameter(GLParameter *parameter_);
  void setReferencePointsTransformation(Eigen::Isometry3f referencePointsTransformation_) { 
    _referencePointsTransformation = referencePointsTransformation_; 
    updateCorrespondenceDrawList();
  }
  void setNumCorrespondences(int numCorrespondences_) { 
    _numCorrespondences = numCorrespondences_; 
    updateCorrespondenceDrawList();
  }
  void setCorrespondences(CorrespondenceVector *correspondences_) { 
    _correspondences = correspondences_;
    updateCorrespondenceDrawList();
  }
  void setReferencePoints(PointVector *referencePoints_) { 
    _referencePoints = referencePoints_; 
    updateCorrespondenceDrawList();
  }
  void setCurrentPoints(PointVector *currentPoints_) { 
    _currentPoints = currentPoints_; 
    updateCorrespondenceDrawList();
  }
  void setStep(int step_) {
    _parameter->setStep(step_);
    updateCorrespondenceDrawList();
  }
  void setLineWidth(float lineWidth_) {
    _parameter->setLineWidth(lineWidth_);
    updateCorrespondenceDrawList();
  }

  virtual void draw();
  void updateCorrespondenceDrawList();
 
 protected:
  Eigen::Isometry3f _referencePointsTransformation;
  GLParameterCorrespondences *_parameter;  
  int _numCorrespondences;
  CorrespondenceVector *_correspondences;
  PointVector *_referencePoints;
  PointVector *_currentPoints;
  GLuint _correspondenceDrawList; 
};  

}

#endif
