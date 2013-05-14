#include "drawable_correspondences.h"
#include "gl_parameter_correspondences.h"

DrawableCorrespondences::DrawableCorrespondences() : Drawable() {
  GLParameterCorrespondences* correspondencesParameter = new GLParameterCorrespondences();
  _parameter = (GLParameter*)correspondencesParameter;
  _numCorrespondences = 0;
  _correspondences = 0;
  _referencePoints = 0;
  _currentPoints = 0;
  _referencePointsTransformation = Eigen::Isometry3f::Identity();
}

DrawableCorrespondences::DrawableCorrespondences(Eigen::Isometry3f transformation_, GLParameter *parameter_,  int numCorrespondences_, PointVector *referencePoints_, PointVector *currentPoints_, CorrespondenceVector *correspondences_) : Drawable(transformation_) {
  setParameter(parameter_);
  _numCorrespondences = numCorrespondences_;
  _correspondences = correspondences_;
  _referencePoints = referencePoints_;
  _currentPoints = currentPoints_;
  _referencePointsTransformation = Eigen::Isometry3f::Identity();
}

bool DrawableCorrespondences::setParameter(GLParameter *parameter_) {
  GLParameterCorrespondences* correspondencesParameter = (GLParameterCorrespondences*)parameter_;
  if (correspondencesParameter == 0) {
    _parameter = 0;
    return false;
  }
  _parameter = parameter_;
  return true;
}

// Drawing function of the class object.
void DrawableCorrespondences::draw() {
  GLParameterCorrespondences* correspondencesParameter = dynamic_cast<GLParameterCorrespondences*>(_parameter);
  if (_referencePoints && 
      _currentPoints && 
      _correspondences && 
      correspondencesParameter && 
      correspondencesParameter->isShown() && 
      correspondencesParameter->lineWidth() > 0.0f) {
    
    glPushMatrix();
    correspondencesParameter->applyGLParameter();
    glBegin(GL_LINES);
    for (int i = 0; i < _numCorrespondences; i += correspondencesParameter->step()) {
      const Correspondence &correspondence = _correspondences->at(i);
      const Point &referencePoint = _referencePointsTransformation * _referencePoints->at(correspondence.referenceIndex);
      const Point &currentPoint = _transformation * _currentPoints->at(correspondence.currentIndex);
      glVertex3f(referencePoint.x(), referencePoint.y(), referencePoint.z());
      glVertex3f(currentPoint.x(), currentPoint.y(), currentPoint.z());
    }
    glEnd();
    glPopMatrix();
  }
}
