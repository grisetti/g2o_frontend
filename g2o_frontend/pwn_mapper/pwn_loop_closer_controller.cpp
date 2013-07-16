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
    // Graph init
    _graph = graph_;
    
    // Projector init
    _sensorOffset = Isometry3f::Identity();
    _sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    _cylindricalPointProjector = new CylindricalPointProjector();
    float angularFov = M_PI;
    float angularResolution = 360.0f / M_PI;
    _cylindricalPointProjector->setAngularFov(angularFov);
    _cylindricalPointProjector->setAngularResolution(angularResolution);
    // For now they are fixed
    _imageRows = angularFov * 2.0f * angularResolution;
    _imageCols = 480;
    
    // Correspondence finder and linearizer init
    _correspondenceFinder = new CorrespondenceFinder();
    _correspondenceFinder->setSize(_imageRows, _imageCols);
    _linearizer = new Linearizer();
    
    // Information matrix calculators init
    _curvatureThreshold = 0.1f;
    _pointInformationMatrixCalculator = new PointInformationMatrixCalculator();
    _normalInformationMatrixCalculator = new NormalInformationMatrixCalculator();
    _pointInformationMatrixCalculator->setCurvatureThreshold(_curvatureThreshold);
    _normalInformationMatrixCalculator->setCurvatureThreshold(_curvatureThreshold);

    // Aligner init
    _minNumInliers = 10000;
    _minError = 10.0f;
    _aligner = new Aligner();
    _aligner->setProjector(_cylindricalPointProjector);
    _aligner->setLinearizer(_linearizer);
    _linearizer->setAligner(_aligner);
    _aligner->setCorrespondenceFinder(_correspondenceFinder);
    _aligner->correspondenceFinder()->setInlierDistanceThreshold(3);
    _aligner->setInnerIterations(1);
    _aligner->setOuterIterations(10);
  }

  bool PWNLoopCloserController::extractAbsolutePrior(Eigen::Isometry3f &priorMean, 
						     Matrix6f &priorInfo, 
						     G2OFrame *currentFrame) {
    VertexSE3 *currentVertex = currentFrame->vertex();
    ImuData *imuData = 0;
    OptimizableGraph::Data *d = currentVertex->userData();
    while(d) {
      ImuData *imuData_ = dynamic_cast<ImuData*>(d);
      if(imuData_) {
	imuData = imuData_;
      }
      d = d->next();
    }
    
    if(imuData) {
      Eigen::Matrix3d R = imuData->getOrientation().matrix();
      Eigen::Matrix3d Omega = imuData->getOrientationCovariance().inverse();
      priorMean.setIdentity();
      priorInfo.setZero();
      for(int c = 0; c < 3; c++)
	for(int r = 0; r < 3; r++)
	  priorMean.linear()(r, c) = R(r, c);
      
      for (int c = 0; c < 3; c++)
	for (int r = 0; r < 3; r++)
	  priorInfo(r + 3, c + 3) = Omega(r, c) * 100000.0f;
      return true;
    }
    return false;
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
      frame->cameraMatrix().setZero();
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

    // Computing information matrices
    frame->pointInformationMatrix().resize(frame->points().size());
    frame->normalInformationMatrix().resize(frame->points().size());
    _pointInformationMatrixCalculator->compute(frame->pointInformationMatrix(), frame->stats(), frame->normals());
    _normalInformationMatrixCalculator->compute(frame->normalInformationMatrix(), frame->stats(), frame->normals());
    
    return true;
  }

  bool PWNLoopCloserController::alignVertexWithPWNData(Isometry3f &transform, 
						       G2OFrame *referenceFrame, 
						       G2OFrame *currentFrame) {

    cerr << "Aligning vertex " << referenceFrame->vertex()->id() << " and " << currentFrame->vertex()->id() << endl;
  
    // Extract initial guess
    Eigen::Isometry3f initialGuess;
    Eigen::Isometry3d delta = referenceFrame->vertex()->estimate().inverse() * currentFrame->vertex()->estimate();
    for(int c = 0; c < 4; c++)
      for(int r = 0; r < 3; r++)
	initialGuess.matrix()(r, c) = delta.matrix()(r, c);
    initialGuess.matrix().col(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    
    Eigen::Isometry3f imuMean;
    Matrix6f imuInfo;
    bool hasImu = this->extractAbsolutePrior(imuMean, imuInfo, currentFrame);
    
    initialGuess.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;    

    // SETTING IDENTITY TO INITIAL GUESS
    initialGuess = Isometry3f::Identity();

    // Setting aligner
    _aligner->clearPriors();
    _aligner->setReferenceFrame(referenceFrame);
    _aligner->setCurrentFrame(currentFrame);
    _aligner->setInitialGuess(initialGuess);
    _aligner->setSensorOffset(_sensorOffset);
    if(hasImu)
      _aligner->addAbsolutePrior(referenceFrame->globalTransform(), imuMean, imuInfo);
    
    // Align
    _aligner->align();  
    transform = _aligner->T();
    
    referenceFrame->save("finalReference.pwn", 1, true);
    currentFrame->save("finalCurrent.pwn", 1, true, _aligner->T());

    if(_aligner->outerIterations() != 0 && 
       (_aligner->inliers() < _minNumInliers || 
	_aligner->error() / _aligner->inliers() > _minError)) {
      cerr << "ALIGNER FAILURE!!!!!!!!!!!!!!!" << endl;
      cerr << "inliers/minimum number of inliers: " << _aligner->inliers() << " / " << _minNumInliers << endl;
      cerr << "error/minimum error: " << _aligner->error() / _aligner->inliers() << " / " << _minError << endl;
      transform.matrix().setZero();
      return false;
    }
  
    // Recondition the rotation to prevent roundoff to accumulate
    Eigen::Matrix3f R = transform.linear();
    Eigen::Matrix3f E = R.transpose() * R;
    E.diagonal().array() -= 1;
    transform.linear() -= 0.5 * R * E;

    if(_aligner->outerIterations() != 0) {
      cout << "Initial guess: " << t2v(initialGuess).transpose() << endl;
      cout << "Transform: " << t2v(_aligner->T()).transpose() << endl;
    }

    // Add edge
    g2o::EdgeSE3* edge = new g2o::EdgeSE3();

    Eigen::Isometry3d iso;
    for(int c = 0; c < 4; c++)
      for(int r = 0; r < 3; r++)
	iso.matrix()(r, c) = _aligner->T().matrix()(r, c);
    edge->setVertex(0,referenceFrame->vertex());
    edge->setVertex(1,currentFrame->vertex());
    edge->setMeasurement(iso);
    edge->setInformation(Eigen::Matrix<double, 6,6>::Identity()*100);
    _graph->addEdge(edge);

    return true;
  }
}
