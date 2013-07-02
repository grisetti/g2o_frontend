#ifndef _PWN_MAPPER_CONTROLLER_H_
#define _PWN_MAPPER_CONTROLLER_H_

#include "g2o_frontend/basemath/bm_se3.h"

#include "g2o_frontend/pwn2/frame.h"
#include "g2o_frontend/pwn2/g2o_frame.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/multipointprojector.h"
#include "g2o_frontend/pwn2/informationmatrixcalculator.h"
#include "g2o_frontend/pwn2/statscalculator.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/aligner.h"
#include "g2o_frontend/pwn2/merger.h"

#include "g2o_frontend/sensor_data/laser_robot_data.h"
#include "g2o_frontend/sensor_data/rgbd_data.h"
#include "g2o_frontend/sensor_data/imu_data.h"
#include "g2o_frontend/sensor_data/pwn_data.h"

#ifdef _PWN_USE_TRAVERSABILITY_
#include "g2o_frontend/traversability/traversability_analyzer.h"
#endif //_PWN_USE_TRAVERSABILITY_

#include "g2o/core/sparse_optimizer.h"

#include <deque>

namespace pwn {

class PWNMapperController {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
 
  PWNMapperController() { graph = 0; }
  ~PWNMapperController() {}
    
  G2OFrame* firstFrame() { return _framesDeque.front(); }
  G2OFrame* lastFrame() { return _framesDeque.back(); }
  int numFrames() { return _framesDeque.size(); }
  size_t maxDequeSize() { return _maxDequeSize; }
  Eigen::Isometry3f alInitialGuess() { return aligner->initialGuess(); }

  inline Eigen::Isometry3f globalTransform() { return globalT; }

  inline int chunkStep() { return _chunkStep; }
  inline float chunkAngle() { return _chunkAngle; }
  inline float chunkDistance() { return _chunkDistance; }
  
  inline void setChunkStep(int chunkStep_) { _chunkStep = chunkStep_; }
  inline void setChunkAngle(float chunkAngle_) { _chunkAngle = chunkAngle_; }
  inline void setChunkDistance(float chunkDistance_) { _chunkDistance = chunkDistance_; }

  inline void setAlMinNumInliers(int al_minNumInliers_) { al_minNumInliers = al_minNumInliers_; }
  inline void setAlMinError(float al_minError_) { al_minError = al_minError_; }

  void setAlOuterIterations(int al_outerIterations_) { al_outerIterations = al_outerIterations_; }
  void setMaxDequeSize(int maxDequeSize_) { _maxDequeSize = maxDequeSize_; }

  void init(g2o::OptimizableGraph* graph_);

  void clear();

  bool addVertex(g2o::VertexSE3* vertex);
  
  bool alignIncrementally();
  
  bool computeTraversability();

  bool addVertex(G2OFrame &frame);

 protected:
  PinholePointProjector *projector;
  MultiPointProjector *multiProjector;
  int numProjectors;
  StatsCalculator *statsCalculator;
  PointInformationMatrixCalculator *pointInformationMatrixCalculator;
  NormalInformationMatrixCalculator *normalInformationMatrixCalculator;
  DepthImageConverter *converter;
  DepthImageConverter *multiConverter;
#ifdef _PWN_USE_TRAVERSABILITY_
  TraversabilityAnalyzer *traversabilityAnalyzer;  
#endif //_PWN_USE_TRAVERSABILITY_
  CorrespondenceFinder *correspondenceFinder;
  Linearizer *linearizer;
  Aligner *aligner;
  Merger *merger;

  std::deque<G2OFrame*> _framesDeque;
  Frame *mergedClouds;

  Isometry3f initialGuess;
  Isometry3f globalT;
  Isometry3f initialPose;
  
  int imageRows, imageCols;
  int reduction;
  float ng_scale;
  float ng_worldRadius;
  float ng_curvatureThreshold;
  int ng_minImageRadius;
  float if_curvatureThreshold;
  int al_innerIterations;
  int al_outerIterations;
  size_t _maxDequeSize;
  
  int _chunkStep;
  float _chunkAngle;
  float _chunkDistance;

  int counter;

  int al_minNumInliers;
  float al_minError;

  DepthImage di;
  MatrixXi ii;
  Frame subScene;

  std::vector<Eigen::Isometry3f> subTrajectory;
  g2o::OptimizableGraph *graph;
  g2o::VertexSE3 *originVertex;
};
 
}

#endif
