#ifndef _G2O_FRAME_H_
#define _G2O_FRAME_H_

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam3d/types_slam3d.h"

namespace pwn {

class G2OFrame : public Frame {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
  G2OFrame(g2o::VertexSE3 *vertex_) {
    _sensorOffset.setIdentity(); 
    _cameraMatrix.setIdentity();
    _vertex = vertex_;
    _previousFrame = 0;
    _globalTransform.setIdentity();
    _previousFrameTransform.setIdentity();
  }  
  virtual ~G2OFrame() {}

  void setPreviousFrame(G2OFrame *previousFrame_) { _previousFrame = previousFrame_; }
  
  Eigen::Isometry3f& sensorOffset() { return _sensorOffset; }    
  Eigen::Matrix3f& cameraMatrix() { return _cameraMatrix; }
  g2o::VertexSE3* vertex() { return _vertex; }
  G2OFrame* previousFrame() { return _previousFrame;}
  Eigen::Isometry3f& globalTransform() { return _globalTransform; }
  Eigen::Isometry3f& previousFrameTransform() { return _previousFrameTransform; }

 protected:
  Eigen::Isometry3f _sensorOffset;
  Eigen::Matrix3f _cameraMatrix;
  g2o::VertexSE3 *_vertex;
  G2OFrame *_previousFrame;
  
  Eigen::Isometry3f _globalTransform; 
  Eigen::Isometry3f _previousFrameTransform;
};

}

#endif
