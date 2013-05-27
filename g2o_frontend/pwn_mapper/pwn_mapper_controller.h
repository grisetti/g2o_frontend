#ifndef _PWN_VIEWER_STATE_H_
#define _PWN_VIEWER_STATE_H_

#include "g2o_frontend/basemath/bm_se3.h"
#include "g2o_frontend/pwn2/frame.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/informationmatrixfinder.h"
#include "g2o_frontend/pwn2/statsfinder.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/aligner.h"

#include "g2o_frontend/sensor_data/laser_robot_data.h"
#include "g2o_frontend/sensor_data/rgbd_data.h"
#include "g2o_frontend/sensor_data/imu_data.h"

#include "g2o_frontend/traversability/traversability_analyzer.h"

#include "g2o/core/sparse_optimizer.h"

#include <deque>

namespace pwn{

  class G2OFrame: public Frame{
    friend class PWNMapperController;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    G2OFrame(g2o::VertexSE3* vertex_){
      _sensorOffset.setIdentity(); 
      _cameraMatrix.setIdentity();
      _vertex = vertex_;
      _previousFrame = 0;
      _globalTransform.setIdentity();
      _previousFrameTransform.setIdentity();
    }

    g2o::VertexSE3* vertex() const {
      return _vertex;
    }

    const Eigen::Isometry3f& sensorOffset() const {
      return  _sensorOffset;
    }
    
    const Eigen::Matrix3f& cameraMatrix() const {
      return _cameraMatrix;
    }

    const Eigen::Isometry3f globalTransform() const {
      return _globalTransform;
    }

    const Eigen::Isometry3f previousFrameTransform() const { return _previousFrameTransform; }

    G2OFrame* previousFrame() const {return _previousFrame;}

  protected:
    Eigen::Isometry3f _sensorOffset;
    Eigen::Matrix3f _cameraMatrix;
    g2o::VertexSE3* _vertex;

    Eigen::Isometry3f _globalTransform, _previousFrameTransform;
    G2OFrame* _previousFrame;
  };


  struct PWNMapperController{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    PWNMapperController();

    // sets up the structures
    void init(g2o::OptimizableGraph* graph_);
    // clears the current state, zeroes the initial guess
    void clear();
    // adds a vertex and all the corresponding observations to the graph
    // returns false if the vertex does not have a rgbd info
    bool addVertex(g2o::VertexSE3* vertex);
    // does the incremental alignment of the current vertex with the previous one(s) 
    bool alignIncrementally();
    bool computeTraversability();

    const std::deque<G2OFrame*> frames() const {return framesDeque;}

  protected:
    // these are the algorithms
    PinholePointProjector* projector;
    StatsFinder* statsFinder;
    PointInformationMatrixFinder* pointInformationMatrixFinder;
    NormalInformationMatrixFinder* normalInformationMatrixFinder;
    DepthImageConverter* converter;
    TraversabilityAnalyzer* traversabilityAnalyzer;
  
    CorrespondenceFinder* correspondenceFinder;
    Linearizer* linearizer;
    Aligner* aligner;

    std::deque<G2OFrame*> framesDeque;

    Isometry3f initialGuess;
    Isometry3f globalT;
  
    int imageRows, imageCols;
    int reduction; // inverse integer scaling for the images
    float ng_scale;
    float ng_worldRadius;
    float ng_curvatureThreshold;
    int ng_minImageRadius;
    float if_curvatureThreshold;
    int al_innerIterations;
    int al_outerIterations;
    size_t maxDequeSize;
    
    g2o::OptimizableGraph* graph;
  };
} // end namespacea
#endif
