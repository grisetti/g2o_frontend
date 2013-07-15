#ifndef DRAWABLE_POSE
#define DRAWABLE_POSE

#include "drawable.h"
#include "gl_parameter_trajectory.h"
#include <Eigen/StdVector>
#include <vector>
#include <iostream>

using namespace std;

namespace pwn {

class DrawableTrajectory : public Drawable {
 public:
  DrawableTrajectory();
  DrawableTrajectory(const Eigen::Isometry3f& transformation_, GLParameter *parameter_, std::vector<Eigen::Isometry3f> *trajectory_);
  virtual ~DrawableTrajectory() { glDeleteLists(_trajectoryDrawList, 1); }

  virtual GLParameter* parameter() { return _parameter; };
  std::vector<Eigen::Isometry3f>* trajectory() { return _trajectory; }
  inline GLuint pyramidDrawList() { return _pyramidDrawList; }
  inline GLuint trajectoryDrawList() { return _trajectoryDrawList; }
  
  virtual bool setParameter(GLParameter *parameter_);
  void setTrajectory(std::vector<Eigen::Isometry3f> *trajectory_) {
    _trajectory = trajectory_;
    updateTrajectoryDrawList();
  }
  void setStep(int step_) {
    _parameter->setStep(step_);
    updateTrajectoryDrawList();
  }
  void setPyramidScale(float pyramidScale_) {
    _parameter->setPyramidScale(pyramidScale_);
    updateTrajectoryDrawList();
  }

  virtual void draw();
  void updateTrajectoryDrawList();
  
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> >* _trajectoryColors;
  
 protected:
  GLParameterTrajectory *_parameter;
  
  std::vector<Eigen::Isometry3f>* _trajectory;
  GLuint _pyramidDrawList;
  GLuint _trajectoryDrawList;
  size_t _trajectorySize;
};  

}

#endif
