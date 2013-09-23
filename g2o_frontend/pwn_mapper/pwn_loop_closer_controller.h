#ifndef _PWN_LOOP_CLOSER_CONTROLLER_H_
#define _PWN_LOOP_CLOSER_CONTROLLER_H_

#include "g2o_frame.h"

#include "g2o/core/sparse_optimizer.h"

#include "g2o_frontend/basemath/bm_se3.h"

#include "g2o_frontend/pwn2/cylindricalpointprojector.h"
#include "g2o_frontend/pwn2/informationmatrixcalculator.h"
#include "g2o_frontend/pwn2/aligner.h"

#include "g2o_frontend/sensor_data/laser_robot_data.h"
#include "g2o_frontend/sensor_data/rgbd_data.h"
#include "g2o_frontend/sensor_data/imu_data.h"
#include "g2o_frontend/sensor_data/pwn_data.h"

using namespace Eigen;
using namespace g2o;

namespace pwn {
  class PWNLoopCloserController {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    // Constructor and destructor
    PWNLoopCloserController(OptimizableGraph *graph_);
    ~PWNLoopCloserController() {}
    
    // Projectors settings methods
    inline void setCylindricalPointProjector(CylindricalPointProjector* const cylindricalPointProjector_) { _cylindricalPointProjector = cylindricalPointProjector_; }
    inline void setImageRows(const int imageRows_) { 
      _imageRows = imageRows_;
      _correspondenceFinder->setImageSize(_imageRows, _imageCols);
      _cylindricalPointProjector->setImageSize(_imageRows, _imageCols);
    }
    inline void setImageCols(const int imageCols_) { 
      _imageCols = imageCols_; 
      _correspondenceFinder->setImageSize(_imageRows, _imageCols);
      _cylindricalPointProjector->setImageSize(_imageRows, _imageCols);
    }
    inline void setSensorOffset(const Isometry3f sensorOffset_) { _sensorOffset = sensorOffset_; }
    
    inline CylindricalPointProjector* cylindricalPointProjector() { return _cylindricalPointProjector; }
    inline int imageRows() const { return _imageRows; }
    inline int imageCols() const { return _imageCols; }
    inline Isometry3f sensorOffset() const { return _sensorOffset; }

    // Correspondence finder and linearizer settings methods
    inline void setCorrespondenceFinder(CorrespondenceFinder* const correspondeceFinder_) { 
      _correspondenceFinder = correspondeceFinder_; 
      _correspondenceFinder->setImageSize(_imageRows, _imageCols);
      _cylindricalPointProjector->setImageSize(_imageRows, _imageCols);
    }
    inline void setLinearizer(Linearizer* const linearizer_) { _linearizer = linearizer_; }

    inline CorrespondenceFinder* correspondenceFinder() { return _correspondenceFinder; }
    inline Linearizer* linearizer() { return _linearizer; }

    // Information matrix calculators settings methods
    inline void setPointInformationMatrixCalculator(PointInformationMatrixCalculator* const pointInformationMatrixCalculator_) { _pointInformationMatrixCalculator = pointInformationMatrixCalculator_; }
    inline void setNormalInformationMatrixCalculator(NormalInformationMatrixCalculator* const normalInformationMatrixCalculator_) { _normalInformationMatrixCalculator = normalInformationMatrixCalculator_; }
    inline void setCurvatureThreshold(const float curvatureThreshold_) { 
      _curvatureThreshold = curvatureThreshold_; 
      _pointInformationMatrixCalculator->setCurvatureThreshold(_curvatureThreshold);
      _normalInformationMatrixCalculator->setCurvatureThreshold(_curvatureThreshold);
    }

    inline PointInformationMatrixCalculator* pointInformationMatrixCalculator() { return _pointInformationMatrixCalculator; }
    inline NormalInformationMatrixCalculator* normalInformationMatrixCalculator() { return _normalInformationMatrixCalculator; }
    inline float curvatureThreshold() const { return _curvatureThreshold; }

    // Aligner settings methods
    inline void setAligner(Aligner* const aligner_) { _aligner = aligner_; }
    inline void setOuterIterations(const int outerIterations_) const { _aligner->setOuterIterations(outerIterations_); }
    inline void setInnerIterations(const int innerIterations_) const { _aligner->setInnerIterations(innerIterations_); }
    inline void setMinNumInliers(const int minNumInliers_) { _minNumInliers = minNumInliers_; }
    inline void setMinError(const float minError_) { _minError = minError_; }

    inline Aligner* aligner() { return _aligner; }
    inline int outerIterations() const { return _aligner->outerIterations(); }
    inline int innerIterations() const { return _aligner->innerIterations(); }
    inline int minNumInliers() const { return _minNumInliers; }
    inline float minError() const { return _minError; }

    // Graph settings methods
    inline void setGraph(OptimizableGraph* const graph_) { _graph = graph_; }    

    inline OptimizableGraph* graph() { return _graph; }

    // Prior extraction method
    bool extractAbsolutePrior(Eigen::Isometry3f &priorMean, 
			      Matrix6f &priorInfo, 
			      G2OFrame *currentFrame);

    // Graph manipulation methods
    bool extractPWNData(G2OFrame *frame) const;
    bool alignVertexWithPWNData(Isometry3f &transform, 
				G2OFrame *referenceFrame, 
				G2OFrame *currentFrame);

  protected:
    // Projectors
    CylindricalPointProjector *_cylindricalPointProjector;
    int _imageRows, _imageCols;
    Isometry3f _sensorOffset;

    // Correspondece finder and linearizer
    CorrespondenceFinder *_correspondenceFinder;
    Linearizer *_linearizer;

    // Information matrix calculators
    PointInformationMatrixCalculator *_pointInformationMatrixCalculator;
    NormalInformationMatrixCalculator *_normalInformationMatrixCalculator;
    float _curvatureThreshold;
    
    // Aligner
    Aligner *_aligner;
    int _minNumInliers;
    float _minError;
     
    // Graph
    OptimizableGraph *_graph;
  };
}

#endif
