#include "pwn_mapper_controller_new.h"

#include <unistd.h>

#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"

#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/types/slam3d/types_slam3d.h"

using namespace std;
using namespace g2o;
using namespace Eigen;

namespace pwn {
  PWNMapperControllerNew::PWNMapperControllerNew(OptimizableGraph *graph_) {
    _graph = graph_;

    // Projector init
    _imageRows = 0;
    _imageCols = 0;
    _scaledImageRows = _imageRows;
    _scaledImageCols = _imageCols;
    _reduction = 2;
    _cameraMatrix << 
      525.0f, 0.0f, 319.5f,
      0.0f, 525.0f, 239.5f,
      0.0f, 0.0f, 1.0f;
    _scaledCameraMatrix << 
      525.0f, 0.0f, 319.5f,
      0.0f, 525.0f, 239.5f,
      0.0f, 0.0f, 1.0f;
    _sensorOffset = Isometry3f::Identity();
    _sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    _projector = new PinholePointProjector();

    // Stats calculator init
    _curvatureThreshold = 0.2f;
    _statsCalculator = new StatsCalculator();
    _statsCalculator->setWorldRadius(0.1f);
    _statsCalculator->setMinImageRadius(10);

    // Information matrix calculators init
    _pointInformationMatrixCalculator = new PointInformationMatrixCalculator();
    _normalInformationMatrixCalculator = new NormalInformationMatrixCalculator();
    _pointInformationMatrixCalculator->setCurvatureThreshold(_curvatureThreshold);
    _normalInformationMatrixCalculator->setCurvatureThreshold(_curvatureThreshold);
    
    // Depth image converter init
    _converter = new DepthImageConverter(_projector, _statsCalculator, 
					 _pointInformationMatrixCalculator, _normalInformationMatrixCalculator);
    _converter->_curvatureThreshold = _curvatureThreshold;
    

    // Correspondence finder and linearizer init
    _correspondenceFinder = new CorrespondenceFinder();
    _linearizer = new Linearizer();

     // Aligner and merger init
    _minNumInliers = 10000;
    _minError = 10.0f;
    _aligner = new Aligner();
    _aligner->setProjector(_projector);
    _aligner->setLinearizer(_linearizer);
    _linearizer->setAligner(_aligner);
    _aligner->setCorrespondenceFinder(_correspondenceFinder);
    _aligner->setInnerIterations(1);
    _aligner->setOuterIterations(10);
    _merger = new Merger();
    _merger->setDepthImageConverter(_converter);

    updateProjector();

    // Frames queue parameters init
    _maxDequeSize = 20;
    _framesDeque.clear();

    // Traversability analyzer init
#ifdef _PWN_USE_TRAVERSABILITY_
    _traversabilityAnalyzer = new TraversabilityAnalyzer(30, 0.04, 0.2, 1);
#endif //_PWN_USE_TRAVERSABILITY_

    // Pwn cloud saving parameters
    _chunkStep = 30;
    _chunkAngle = M_PI_2;
    _chunkDistance = 1.0f;
  }

  bool PWNMapperControllerNew::extractRelativePrior(Eigen::Isometry3f &priorMean, 
						    Matrix6f &priorInfo, 
						    G2OFrame *reference, 
						    G2OFrame *current) {
    VertexSE3 *referenceVertex = reference->vertex();
    VertexSE3 *currentVertex = current->vertex();
    bool priorFound = false;
    priorInfo.setZero();
    for(HyperGraph::EdgeSet::const_iterator it = referenceVertex->edges().begin(); it != referenceVertex->edges().end(); it++) {
      const EdgeSE3 *e = dynamic_cast<const EdgeSE3*>(*it);
      if(e->vertex(0) == referenceVertex && e->vertex(1) == currentVertex) {
	priorFound=true;
	for(int c = 0; c < 6; c++)
	  for(int r = 0; r < 6; r++)
	    priorInfo(r, c) = e->information()(r, c);
      
	for(int c = 0; c < 4; c++)
	  for(int r = 0; r < 3; r++)
	    priorMean.matrix()(r, c) = e->measurement().matrix()(r, c);
	priorMean.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
      }
    }
    return priorFound;
  }

  bool PWNMapperControllerNew::extractAbsolutePrior(Eigen::Isometry3f &priorMean, 
						    Matrix6f &priorInfo, 
						    G2OFrame *current) {
    VertexSE3 *currentVertex = current->vertex();
    ImuData *imuData = 0;
    OptimizableGraph::Data *d = currentVertex->userData();
    while(d) {
      ImuData *imuData_ = dynamic_cast<ImuData*>(d);
      if (imuData_) {
	imuData = imuData_;
      }
      d = d->next();
    }
  
    if (imuData) {
      Eigen::Matrix3d R = imuData->getOrientation().matrix();
      Eigen::Matrix3d Omega = imuData->getOrientationCovariance().inverse();
      priorMean.setIdentity();
      priorInfo.setZero();
      for(int c = 0; c < 3; c++)
	for(int r = 0; r < 3; r++)
	  priorMean.linear()(r, c) = R(r, c);
    
      for (int c = 0; c < 3; c++)
	for (int r = 0; r < 3; r++)
	  priorInfo(r+3, c+3)=Omega(r, c) * 100000;
      return true;
    }
    return false;
  }

  void PWNMapperControllerNew::clear() {
    while(!_framesDeque.empty()) {
      delete _framesDeque.front();
      _framesDeque.pop_front();
    }
  }

  bool PWNMapperControllerNew::addVertex(VertexSE3 *v) {
  OptimizableGraph::Data *d = v->userData();
  RGBDData *rgbdData = 0;
  while(d && !rgbdData) {
    rgbdData = dynamic_cast<RGBDData*>(d);
    d = d->next();
  }
    
  if(!rgbdData)
    return false;
  
  cerr << "Adding vertex: "<< v->id() << endl;
  int paramIndex = rgbdData->paramIndex();
  // retrieve from the graph the parameter given the index  
  g2o::Parameter *_cameraParameter = _graph->parameter(paramIndex);
  // attempt a cast to a parameter camera  
  ParameterCamera *cameraParameter = dynamic_cast<ParameterCamera*>(_cameraParameter);
  if(!_cameraParameter) {
    cerr << "Could not find a valid camera" << endl;
    return false;
  }
    
  // We got the parameter
  Eigen::Matrix3f cameraMatrix;
  Eigen::Isometry3f sensorOffset;
  cameraMatrix.setZero();
  
  // Get the sensor offset andcamera matrix
  int cmax = 4;
  int rmax = 3;
  for(int c = 0; c < cmax; c++) {
    for(int r = 0; r < rmax; r++) {
      _sensorOffset.matrix()(r, c) = cameraParameter->offset()(r, c);
      if(c < 3)
	_cameraMatrix(r, c) = cameraParameter->Kcam()(r, c);
    }
  }
  
  // HAKK: setting sensor offset and camera matrix to fixed values
  _cameraMatrix << 
    525.0f, 0.0f, 319.5f,
    0.0f, 525.0f, 239.5f,
    0.0f, 0.0f, 1.0f;
  
  _sensorOffset = Isometry3f::Identity();
  _sensorOffset.translation() = Vector3f(0.15f, 0.0f, 0.05f);
  Quaternionf quat = Quaternionf(0.5f, -0.5f, 0.5f, -0.5f);
  _sensorOffset.linear() = quat.toRotationMatrix();
  _sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

  updateProjector();

  // Get the filename
  std::string filename = rgbdData->baseFilename() + "_depth.pgm";

  // Read the depth image
  if (!_depthImage.load(filename.c_str(), true)){
    cerr << "Impossible to load image " << filename << endl;
    return false;
  }
    
  // Scale the depth image
  DepthImage::scale(_scaledDepthImage, _depthImage, _reduction);

  // Create the frame and set the parameters
  G2OFrame *frame = new G2OFrame(v);
  frame->cameraMatrix() = _cameraMatrix;
  frame->sensorOffset() = _sensorOffset;
    
  G2OFrame *previousFrame = 0;
  if(!_framesDeque.empty()) 
    previousFrame = _framesDeque.back();
    
  // If it is the first vertex, we put the imu as prior if avalable
  if(!previousFrame) {
    frame->globalTransform().setIdentity();
    frame->previousFrameTransform().setIdentity();
    Eigen::Isometry3f priorMean;
    Matrix6f priorInfo;
    bool hasImu = extractAbsolutePrior(priorMean, priorInfo, frame);
    if(hasImu) {
      cerr << "Found an IMU for the first vertex" << endl;
      frame->globalTransform().linear() = priorMean.linear();
    }
  }

  frame->setPreviousFrame(previousFrame);

  // Compute the stats for the current cloud
  _converter->compute(*frame, _scaledDepthImage, _sensorOffset, true);
  _framesDeque.push_back(frame);

  // Keep at most maxDequeSize elements in the queue
  while(_framesDeque.size() > _maxDequeSize) {
    G2OFrame *frame = _framesDeque.front();
    _framesDeque.pop_front();
    _framesDeque.front()->setPreviousFrame(0);
    delete frame;
  }

  return true;
}

#ifdef _PWN_USE_TRAVERSABILITY_
  bool PWNMapperControllerNew::computeTraversability() { 
    G2OFrame *current = _framesDeque.back();
    if(!current)
      return false;
    G2OFrame globalCurrentFrame(*current);
    Eigen::Isometry3f globalTransform = current->globalTransform();
    globalTransform.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    globalCurrentFrame.transformInPlace(globalTransform);
    traversabilityAnalyzer->createTraversabilityVector(globalCurrentFrame.points(), 
						       globalCurrentFrame.normals(), 
						       globalCurrentFrame.traversabilityVector()); 
    return true;
  }
#endif //_PWN_USE_TRAVERSABILITY_

  void PWNMapperControllerNew::updateProjector() {
    // Compute the reduced camera matrix and image size
    float scale = 1.0f / _reduction;
    _scaledCameraMatrix = _cameraMatrix * scale;
    _scaledCameraMatrix(2, 2) = 1.0f;
    _scaledImageRows = _imageRows / _reduction;
    _scaledImageCols = _imageCols / _reduction;
    
    // Set the camera matrix to the pinhole point projector
    _projector->setCameraMatrix(_scaledCameraMatrix);

    // Set image size to the correspondence finder
    _correspondenceFinder->setSize(_scaledImageRows, _scaledImageCols);

    // Set image size to the merger
    _merger->setImageSize(_scaledImageRows, _scaledImageCols);
  }
}
