#ifndef _PWN_LOOP_CLOSER_CONTROLLER_H_
#define _PWN_LOOP_CLOSER_CONTROLLER_H_

#include "g2o_frame.h"

#include "g2o/core/sparse_optimizer.h"

#include "g2o_frontend/basemath/bm_se3.h"

#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/multipointprojector.h"
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
    inline void setPinholePointProjector(PinholePointProjector* const pinholePointProjector_) { 
      _pinholePointProjector = pinholePointProjector_;
      updateProjectors();
    }
    inline void setMultiPointProjector(MultiPointProjector* const multiPointProjector_) { _multiPointProjector = multiPointProjector_; }
    inline void setNumProjectors(const int numProjectors_) { 
      _numProjectors = numProjectors_; 
      updateProjectors();
    }
    inline void setImageRows(const int imageRows_) { 
      _imageRows = imageRows_; 
      updateProjectors();
    }
    inline void setImageCols(const int imageCols_) { 
      _imageCols = imageCols_; 
      updateProjectors();
    }
    inline void setReduction(const int reduction_) { 
      _reduction = reduction_; 
      updateProjectors();
    }
    inline void setCameraMatrix(const Matrix3f cameraMatrix_) { 
      _cameraMatrix = cameraMatrix_;
      updateProjectors();
    }
    inline void setSensorOffset(const Isometry3f sensorOffset_) {
      _sensorOffset = sensorOffset_;
      updateProjectors();
    }
    
    inline PinholePointProjector* pinholePointProjector() { return _pinholePointProjector; }
    inline MultiPointProjector* multiPointProjector() { return _multiPointProjector; }
    inline int numProjectors() const { return _numProjectors; }
    inline int imageRows() const { return _imageRows; }
    inline int imageCols() const { return _imageCols; }
    inline int reduction() const { return _reduction; }
    inline Matrix3f cameraMatrix() const { return _cameraMatrix; }
    inline Isometry3f sensorOffset() const { return _sensorOffset; }

    // Correspondence finder and linearizer settings methods
    inline void setCorrespondenceFinder(CorrespondenceFinder* const correspondeceFinder_) { _correspondenceFinder = correspondeceFinder_; }
    inline void setLinearizer(Linearizer* const linearizer_) { _linearizer = linearizer_; }

    inline CorrespondenceFinder* correspondenceFinder() { return _correspondenceFinder; }
    inline Linearizer* linearizer() { return _linearizer; }

    // Information matrix calculators
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

    // Alignement settings methods
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
    PinholePointProjector *_pinholePointProjector;
    MultiPointProjector *_multiPointProjector;
    int _numProjectors, _imageRows, _imageCols, _scaledImageRows, _scaledImageCols, _reduction;
    Matrix3f _cameraMatrix, _scaledCameraMatrix;
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

  private:
    void updateProjectors();
  };
}

#endif
