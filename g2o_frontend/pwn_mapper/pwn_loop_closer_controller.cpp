#include "pwn_loop_closer_controller.h"

#include <unistd.h>

#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam3d/types_slam3d.h"

using namespace std;
using namespace Eigen;
using namespace g2o;

namespace pwn {
  
  PWNLoopCloserController::PWNLoopCloserController(OptimizableGraph *graph_) {
    _graph = graph_;
    
    // Projectors init
    _numProjectors = 4;
    _imageRows = 0;
    _imageCols = 0;
    _reduction = 2;
    _cameraMatrix << 
      525.0f, 0.0f, 319.5f,
      0.0f, 525.0f, 239.5f,
      0.0f, 0.0f, 1.0f;
    _sensorOffset = Isometry3f::Identity();
    _sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    _pinholePointProjector = new PinholePointProjector();
    _multiPointProjector = new MultiPointProjector();
    updateProjectors();

    // Correspondence finder and linearizer init
    _correspondenceFinder = new CorrespondenceFinder();
    _linearizer = new Linearizer() ;
    
    // Aligner init
    _minNumInliers = 10000;
    _minError = 10.0f;
    _aligner = new Aligner();
    _aligner->setProjector(_multiPointProjector);
    _aligner->setLinearizer(_linearizer);
    _linearizer->setAligner(_aligner);
    _aligner->setCorrespondenceFinder(_correspondenceFinder);
    _aligner->setInnerIterations(1);
    _aligner->setOuterIterations(10);
  }

  bool PWNLoopCloserController::extractPWNData(G2OFrame *frame) const {
    // Get the list of data from the input vertex and check if one of them it's a PWNData
    OptimizableGraph::Data *d = frame->vertex()->userData();
    PWNData *pwnData = 0;
    while(d && !pwnData) {
      pwnData = dynamic_cast<PWNData*>(d);
      d = d->next();
    }
    
    if(!pwnData)
      return false;
  
    // Get the .pwn filename
    std::string fname = pwnData->filename();
    // Get the global transformation of the cloud
    Eigen::Isometry3f originPose = Eigen::Isometry3f::Identity();
    originPose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

    // Load cloud from the file
    if(frame->load(originPose, fname.c_str())) {      
      // Set frame parameters
      frame->cameraMatrix() = _cameraMatrix;
      frame->sensorOffset() = _sensorOffset;    
      frame->globalTransform() = originPose;
      frame->globalTransform().matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
      frame->previousFrameTransform().setIdentity();
      frame->previousFrameTransform().matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
      frame->setPreviousFrame(0);
    }
    else {
      return false;
    }

    return true;
  }

  bool PWNLoopCloserController::alignVertexWithPWNData(Isometry3f &transform, 
						       G2OFrame *referenceFrame, 
						       G2OFrame *currentFrame) const {
    return true;
  }

  void PWNLoopCloserController::updateProjectors() {
    // Clear the list of projectors
    _multiPointProjector->clearProjectors();

    // Compute the reduced camera matrix and image size
    float scale = 1.0f / _reduction;
    _cameraMatrix *= scale;
    _cameraMatrix(2, 2) = 1.0f;
    _imageRows = _imageRows / _reduction;
    _imageCols = _imageCols / _reduction;
    
    // Set the camera matrix to the pinhole point projector
    _pinholePointProjector->setCameraMatrix(_cameraMatrix);
    
    // Create the projectors for the multi projector
    float angleStep = 2.0f * M_PI / _numProjectors;

    for(int i = 0; i < _numProjectors; i++) {
      Isometry3f currentSensorOffset = _sensorOffset;
      if(i > 0)
	currentSensorOffset.linear() =  AngleAxisf(i * angleStep, Vector3f::UnitZ()) * _sensorOffset.linear();
      currentSensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    
      _multiPointProjector->addPointProjector(_pinholePointProjector, currentSensorOffset, 
					      _imageRows, _imageCols);
    }
  }

}
