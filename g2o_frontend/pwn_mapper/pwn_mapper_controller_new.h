#ifndef _PWN_MAPPER_CONTROLLER_NEW_H_
#define _PWN_MAPPER_CONTROLLER_NEW_H_

#include <deque>

#include "g2o/core/sparse_optimizer.h"

#include "g2o_frame.h"

#include "g2o_frontend/basemath/bm_se3.h"

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

namespace pwn {

  class PWNMapperControllerNew {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    PWNMapperControllerNew(g2o::OptimizableGraph *graph_);
    ~PWNMapperControllerNew() {}

    // Projector settings methods
    inline void setProjector(PinholePointProjector* const projector_) { 
      _projector = projector_;
      updateProjector();
    }
    inline void setImageRows(const int imageRows_) { 
      _imageRows = imageRows_; 
      updateProjector();
    }
    inline void setImageCols(const int imageCols_) { 
      _imageCols = imageCols_; 
      updateProjector();
    }
    inline void setReduction(const int reduction_) { 
      _reduction = reduction_; 
      updateProjector();
    }
    inline void setCameraMatrix(const Matrix3f cameraMatrix_) { 
      _cameraMatrix = cameraMatrix_;
      updateProjector();
    }
    inline void setSensorOffset(const Isometry3f sensorOffset_) {
      _sensorOffset = sensorOffset_;
      updateProjector();
    }

    inline PinholePointProjector* projector() { return _projector; }
    inline int imageRows() const { return _imageRows; }
    inline int imageCols() const { return _imageCols; }
    inline int reduction() const { return _reduction; }
    inline Matrix3f cameraMatrix() const { return _cameraMatrix; }
    inline Isometry3f sensorOffset() const { return _sensorOffset; }

    // Stats calculator settings methods
    inline void setStatsCalculator(StatsCalculator* const statsCalculator_) { _statsCalculator = statsCalculator_; }
    inline void setCurvatureThreshold(const float curvatureThreshold_) { 
      _curvatureThreshold = curvatureThreshold_; 
      _pointInformationMatrixCalculator->setCurvatureThreshold(_curvatureThreshold);
      _normalInformationMatrixCalculator->setCurvatureThreshold(_curvatureThreshold);
    }

    inline StatsCalculator* statsCalculator() { return _statsCalculator; }
    inline float curvatureThreshold() const { return _curvatureThreshold; }

    // Information matrix calculators settings methods
    inline void setPointInformationMatrixCalculator(PointInformationMatrixCalculator* const pointInformationMatrixCalculator_) { _pointInformationMatrixCalculator = pointInformationMatrixCalculator_; }
    inline void setNormalInformationMatrixCalculator(NormalInformationMatrixCalculator* const normalInformationMatrixCalculator_) { _normalInformationMatrixCalculator = normalInformationMatrixCalculator_; }

    inline PointInformationMatrixCalculator* pointInformationMatrixCalculator() { return _pointInformationMatrixCalculator; }
    inline NormalInformationMatrixCalculator* normalInformationMatrixCalculator() { return _normalInformationMatrixCalculator; }

    // Depth image converter settings methods
    inline void setConverter(DepthImageConverter* const converter_) { _converter = converter_; }
    inline void setDepthImage(DepthImage* const depthImage_) { _depthImage = *depthImage_; }

    inline DepthImageConverter* converter() { return _converter; }
    inline DepthImage* depthImage() { return &_depthImage; }

    // Correspondence finder and linearizer settings methods
    inline void setCorrespondenceFinder(CorrespondenceFinder* const correspondeceFinder_) { 
      _correspondenceFinder = correspondeceFinder_; 
      updateProjector();
    }
    inline void setLinearizer(Linearizer* const linearizer_) { _linearizer = linearizer_; }

    inline CorrespondenceFinder* correspondenceFinder() { return _correspondenceFinder; }
    inline Linearizer* linearizer() { return _linearizer; }

    // Aligner and merger settings methods
    inline void setAligner(Aligner* const aligner_) { _aligner = aligner_; }
    inline void setMerger(Merger* const merger_) { 
      _merger = merger_;
      updateProjector();
    }
    inline void setOuterIterations(const int outerIterations_) const { _aligner->setOuterIterations(outerIterations_); }
    inline void setInnerIterations(const int innerIterations_) const { _aligner->setInnerIterations(innerIterations_); }
    inline void setMinNumInliers(const int minNumInliers_) { _minNumInliers = minNumInliers_; }
    inline void setMinError(const float minError_) { _minError = minError_; }

    inline Aligner* aligner() { return _aligner; }
    inline Merger* merger() { return _merger; }
    inline int outerIterations() const { return _aligner->outerIterations(); }
    inline int innerIterations() const { return _aligner->innerIterations(); }
    inline int minNumInliers() const { return _minNumInliers; }
    inline float minError() const { return _minError; }

    // Traversability analyzer settings methods
#ifdef _PWN_USE_TRAVERSABILITY_
    inline void setTraversabilityAnalyzer(TraversabilityAnalyzer* const traversabilityAnalyzer_) { _traversabilityAnalyzer = traversabilityAnalyzer_; }

    inline TraversabilityAnalyzer* traversabilityAnalyzer() { return _traversabilityAnalyzer; }
#endif //_PWN_USE_TRAVERSABILITY_
    
    // Frames queue parameters settings methods
    inline void setFramesDeque(std::deque<G2OFrame*>* const framesDeque_) { _framesDeque = *framesDeque_; }
    inline void setMaxDequeSize(const size_t maxDequeSize_) { _maxDequeSize = maxDequeSize_; }

    inline std::deque<G2OFrame*>* framesDeque() { return &_framesDeque; }
    inline size_t maxDequeSize() { return _maxDequeSize; }
    
    int framesDequeSize() { return _framesDeque.size(); }
    G2OFrame* framesDequeFirstFrame() { return _framesDeque.front(); }
    G2OFrame* framesDequeLastFrame() { return _framesDeque.back(); }
    
    // Pwn cloud saving parameters settings methods
    inline void setChunkStep(const int chunkStep_) { _chunkStep = chunkStep_; }
    inline void setChunkAngle(const float chunkAngle_) { _chunkAngle = chunkAngle_; }
    inline void setChunkDistance(const float chunkDistance_) { _chunkDistance = chunkDistance_; }

    inline size_t chunkStep() { return _chunkStep; }
    inline size_t chunkAngle() { return _chunkAngle; }
    inline size_t chunkDistance() { return _chunkDistance; }

    // Graph settings methods
    inline void setGraph(OptimizableGraph* const graph_) { _graph = graph_; }    

    inline OptimizableGraph* graph() { return _graph; }

    // Prior extraction methods
    bool extractRelativePrior(Eigen::Isometry3f &priorMean, 
			      Matrix6f &priorInfo, 
			      G2OFrame *reference, 
			      G2OFrame *current);
    bool extractAbsolutePrior(Eigen::Isometry3f &priorMean, 
			      Matrix6f &priorInfo, 
			      G2OFrame *currentFrame);

    // Graph manipulation methods
    void clear();
    
    bool addVertex(g2o::VertexSE3* vertex);

    bool computeTraversability();

   protected:
    // Projector
    PinholePointProjector *_projector;
    int _imageRows, _imageCols, _scaledImageRows, _scaledImageCols, _reduction;
    Matrix3f _cameraMatrix, _scaledCameraMatrix;
    Eigen::Isometry3f _sensorOffset;

    // Stats calculator
    StatsCalculator *_statsCalculator;
    float _curvatureThreshold;

    // Information matrix calculators
    PointInformationMatrixCalculator *_pointInformationMatrixCalculator;
    NormalInformationMatrixCalculator *_normalInformationMatrixCalculator;

    // Depth image converter
    DepthImageConverter *_converter;
    DepthImage _depthImage, _scaledDepthImage;

    // Correspondece finder and linearizer
    CorrespondenceFinder *_correspondenceFinder;
    Linearizer *_linearizer;

    // Aligner and merger
    Aligner *_aligner;
    Merger *_merger;
    int _minNumInliers;
    float _minError;

    // Traversability analyzer
#ifdef _PWN_USE_TRAVERSABILITY_
    TraversabilityAnalyzer *traversabilityAnalyzer;  
#endif //_PWN_USE_TRAVERSABILITY_

    // Frames queue parameters
    std::deque<G2OFrame*> _framesDeque;
    size_t _maxDequeSize;

    // Pwn cloud saving parameters
    int _chunkStep;
    float _chunkAngle;
    float _chunkDistance;
   
    // Graph
    g2o::OptimizableGraph *_graph; 

  private:
    void updateProjector();
  };
}

#endif
