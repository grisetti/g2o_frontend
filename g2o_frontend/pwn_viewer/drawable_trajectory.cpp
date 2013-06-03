#include "drawable_trajectory.h"
#include "g2o/stuff/opengl_primitives.h"

#include <iostream>
using namespace std;

namespace pwn {

DrawableTrajectory::DrawableTrajectory() : Drawable() {
  _parameter = 0;
  _trajectory = 0;
  _pyramidDrawList = glGenLists(1);
  _trajectoryDrawList = glGenLists(1);
  glNewList(_pyramidDrawList, GL_COMPILE);
  g2o::opengl::drawPyramid(1.0f, 1.0f);
  glEndList();
  _trajectorySize = 0;
  updateTrajectoryDrawList();
}

  DrawableTrajectory::DrawableTrajectory(const Eigen::Isometry3f& transformation_, GLParameter *parameter_, 
			     std::vector<Eigen::Isometry3f>* trajectory_) : Drawable(transformation_) {
  setParameter(parameter_);
  _trajectory = trajectory_;
  _pyramidDrawList = glGenLists(1);
  _trajectoryDrawList = glGenLists(1);
  glNewList(_pyramidDrawList, GL_COMPILE);
  g2o::opengl::drawPyramid(1.0f, 1.0f);
  glEndList();
  if(_trajectory != 0)
    _trajectorySize = _trajectory->size();
  else
    _trajectorySize = 0;
  updateTrajectoryDrawList();
}

bool DrawableTrajectory::setParameter(GLParameter *parameter_) {
  GLParameterTrajectory *trajectoryParameter = (GLParameterTrajectory*)parameter_;
  if(trajectoryParameter == 0) {
    _parameter = 0;
    return false;
  }
  _parameter = trajectoryParameter;
  return true;
}

void DrawableTrajectory::draw() {
  GLParameterTrajectory *trajectoryParameter = dynamic_cast<GLParameterTrajectory*>(_parameter);
  if(_trajectory &&
     trajectoryParameter && 
     trajectoryParameter->show() && 
     trajectoryParameter->pyramidScale() > 0.0f) {
    if(_trajectorySize != _trajectory->size())
      updateTrajectoryDrawList();
    glPushMatrix();
    glMultMatrixf(_transformation.data());
    trajectoryParameter->applyGLParameter();
    glCallList(_trajectoryDrawList);
    glPopMatrix();
  }
}

void DrawableTrajectory::updateTrajectoryDrawList() {
  GLParameterTrajectory *trajectoryParameter = dynamic_cast<GLParameterTrajectory*>(_parameter);
  glNewList(_trajectoryDrawList, GL_COMPILE);    
  if(_trajectory && 
     trajectoryParameter && 
     trajectoryParameter->show() && 
     trajectoryParameter->pyramidScale() > 0.0f) {
    for(size_t i = 0; i < _trajectory->size(); i += trajectoryParameter->step()) {
      glPushMatrix();
      glMultMatrixf(_trajectory->at(i).data());
      glRotatef(90.0f, 0.0f, 1.0f, 0.0f);        
      glScalef(trajectoryParameter->pyramidScale() ,trajectoryParameter->pyramidScale(), trajectoryParameter->pyramidScale() * 2.0f);
      glCallList(_pyramidDrawList);
      glPopMatrix();
    }
  }
  glEndList();
}
  
}
