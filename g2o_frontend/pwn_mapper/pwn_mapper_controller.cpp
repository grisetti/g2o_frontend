#include "pwn_mapper_controller.h"

#include <unistd.h>

#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"

#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/types/slam3d/types_slam3d.h"

#include "g2o_frontend/pwn_utils/pwn_utils.h"

using namespace std;
using namespace g2o;
using namespace Eigen;

namespace pwn {
  PWNMapperController::PWNMapperController(OptimizableGraph *graph_) {
    _graph = graph_;

    // Projector init
    _imageRows = 0;
    _imageCols = 0;
    _scaledImageRows = _imageRows;
    _scaledImageCols = _imageCols;
    _reduction = 2;
    _cameraMatrix.setIdentity();
    _scaledCameraMatrix.setIdentity();

    // _cameraMatrix << 
    //   525.0f, 0.0f, 319.5f,
    //   0.0f, 525.0f, 239.5f,
    //   0.0f, 0.0f, 1.0f;
    // _scaledCameraMatrix << 
    //   525.0f, 0.0f, 319.5f,
    //   0.0f, 525.0f, 239.5f,
    //   0.0f, 0.0f, 1.0f;
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
    _correspondenceFinder->setInlierDistanceThreshold(0.3);
    _correspondenceFinder->setInlierNormalAngularThreshold(0.5);

    _linearizer = new Linearizer();

    // Voxel calculator init
    _voxelCalculator = new VoxelCalculator();

    // Aligner and merger init
    _minNumInliers = 1000;
    _minError = 5.0f;
    _aligner = new Aligner();
    _aligner->setProjector(_projector);
    _aligner->setLinearizer(_linearizer);
    _linearizer->setAligner(_aligner);
    _aligner->setCorrespondenceFinder(_correspondenceFinder);
    _aligner->setInnerIterations(1);
    _aligner->setOuterIterations(10);
    _scene = new Frame();
    _subScene = new Frame();
    _initialScenePose = Isometry3f::Identity();
    _initialScenePose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
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
    _pwnSaving = false;
    _chunkStep = 30;
    _chunkAngle = M_PI_2;
    _chunkDistance = 1.0f;
  }

  bool PWNMapperController::extractRelativePrior(Eigen::Isometry3f &priorMean, 
						    Matrix6f &priorInfo, 
						    G2OFrame *reference, 
						    G2OFrame *current) {
    VertexSE3 *referenceVertex = reference->vertex();
    VertexSE3 *currentVertex = current->vertex();
    assert(referenceVertex);
    assert(currentVertex);

    bool priorFound = false;
    priorInfo.setZero();
    for(HyperGraph::EdgeSet::const_iterator it = referenceVertex->edges().begin(); it != referenceVertex->edges().end(); it++) {
      const EdgeSE3 *e = dynamic_cast<const EdgeSE3*>(*it);
      if(e && e->vertex(0) == referenceVertex && e->vertex(1) == currentVertex) {
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

  bool PWNMapperController::extractAbsolutePrior(Eigen::Isometry3f &priorMean, 
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

  void PWNMapperController::clear() {
    while(!_framesDeque.empty()) {
      delete _framesDeque.front();
      _framesDeque.pop_front();
    }
  }

  bool PWNMapperController::addVertex(VertexSE3 *v) {
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
    // Get the sensor offset and camera matrix
    _cameraMatrix.setZero();
    int cmax = 4;
    int rmax = 3;
    for(int c = 0; c < cmax; c++) {
      for(int r = 0; r < rmax; r++) {
	_sensorOffset.matrix()(r, c) = cameraParameter->offset()(r, c);
	if(c < 3)
	  _cameraMatrix(r, c) = cameraParameter->Kcam()(r, c);
      }
    }
  
  
    _sensorOffset = Isometry3f::Identity();
    _sensorOffset.translation() = Vector3f(0.15f, 0.0f, 0.05f);
    Quaternionf quaternion;
    //xyzToQuat(quaternion, -0.579275, 0.56288, -0.41087); // segway_02   
    quaternion = Quaternionf(0.5f, -0.5f, 0.5f, -0.5f);
    _sensorOffset.linear() = quaternion.toRotationMatrix();
    _sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
   
    updateProjector();

    // Get the filename
    std::string filename = rgbdData->baseFilename();// + "_depth.pgm";
    cerr << "loading  " << filename << endl;
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
    _converter->compute(*frame, _scaledDepthImage, _sensorOffset, false);
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

  bool PWNMapperController::addVertex(G2OFrame &frame) {
    OptimizableGraph::Data *d = frame.vertex()->userData();
    RGBDData *rgbdData = 0;
    while(d && !rgbdData) {
      rgbdData = dynamic_cast<RGBDData*>(d);
      d = d->next();
    }
    
    if(!rgbdData)
      return false;
  
    int paramIndex = rgbdData->paramIndex();
    g2o::Parameter *_cameraParameter = _graph->parameter(paramIndex);
    ParameterCamera *cameraParameter = dynamic_cast<ParameterCamera*>(_cameraParameter);
    if(!cameraParameter) {
      cerr << "Could not find a valid camera" << endl;
      return false;
    }
    
    // We got the parameter  
    // Get the sensor offset and camera matrix
    _cameraMatrix.setZero();
    int cmax = 4;
    int rmax = 3;
    for(int c = 0; c < cmax; c++) {
      for(int r = 0; r < rmax; r++) {
	_sensorOffset.matrix()(r, c) = cameraParameter->offset()(r, c);
	if(c < 3)
	  _cameraMatrix(r, c) = cameraParameter->Kcam()(r, c);
      }
    }
  
    // // HAKK: setting sensor offset and camera matrix to fixed values
    // _cameraMatrix << 
    //   525.0f, 0.0f, 319.5f,
    //   0.0f, 525.0f, 239.5f,
    //   0.0f, 0.0f, 1.0f;
  
    _sensorOffset = Isometry3f::Identity();
    _sensorOffset.translation() = Vector3f(0.15f, 0.0f, 0.05f);
    Quaternionf quaternion;
    //xyzToQuat(quaternion, -0.579275, 0.56288, -0.41087); // segway_02   
    quaternion = Quaternionf(0.5f, -0.5f, 0.5f, -0.5f);
    _sensorOffset.linear() = quaternion.toRotationMatrix();
    _sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

    updateProjector();

    // Read the depth image and scale it
    //std::string filename = rgbdData->baseFilename(); + "_depth.pgm";
    std::string filename = rgbdData->baseFilename(); //+ "_depth.pgm";

    if(!_depthImage.load(filename.c_str(), true)) {
      cerr << "No depth image loaded." << endl;
      return false;
    }
    
    DepthImage::scale(_scaledDepthImage, _depthImage, _reduction);

    frame.cameraMatrix() = _cameraMatrix;
    frame.sensorOffset() = _sensorOffset;
    
    frame.globalTransform().setIdentity();
    frame.previousFrameTransform().setIdentity();
    frame.setPreviousFrame(0);

    _converter->compute(frame, _scaledDepthImage, _sensorOffset, false);

    return true;
  }

  bool PWNMapperController::alignIncrementally() {
    if(_framesDeque.size() < 2)
      return false;

    // Take the frames to align
    G2OFrame *current = _framesDeque.back();
    G2OFrame *reference = current->previousFrame();

    if(_pwnSaving) {
      // Merge the scene with the new added cloud
      _scene->add(*reference, _initialScenePose.inverse() * reference->globalTransform());
      _sceneVerteces.push_back(reference->vertex());
      _merger->merge(_scene, _initialScenePose.inverse() * reference->globalTransform() * _sensorOffset);
    }

    cerr << "********************** Aligning vertex " << current->vertex()->id() << " **********************" << endl;
  
    Eigen::Isometry3f initialGuess;
    // Computing initial guess based on the frame positions
    Eigen::Isometry3d delta = reference->vertex()->estimate().inverse() * current->vertex()->estimate();
    for(int c = 0; c < 4; c++)
      for(int r = 0; r < 3; r++)
	initialGuess.matrix()(r, c) = delta.matrix()(r, c);

    Eigen::Isometry3f odometryMean;
    Matrix6f odometryInfo;
    bool hasOdometry = extractRelativePrior(odometryMean, odometryInfo, reference, current);
    if (hasOdometry) {
      initialGuess = odometryMean;
      odometryInfo.block<3,3>(0,0) *= 1e3;
    }
    // Force a prior
    else { 
      hasOdometry = true;
      odometryMean = initialGuess;
      odometryInfo.setIdentity();
      odometryInfo.block<3,3>(0,0) *= 1e3;
    }

    //odometryInfo.block *= 1e9;
    Eigen::Isometry3f imuMean;
    Matrix6f imuInfo;
    bool hasImu = extractAbsolutePrior(imuMean, imuInfo, current);
    initialGuess.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

    if(_pwnSaving) {
      // Compute the reference subscene
      if(_indexImage.cols() != _scaledImageCols || _indexImage.rows() != _scaledImageRows) 
      	_indexImage.resize(_scaledImageRows, _scaledImageCols);
      _projector->setTransform(_initialScenePose.inverse() * reference->globalTransform() * _sensorOffset);
      _projector->project(_indexImage, _depthImage, _scene->points());
      
      _converter->compute(*_subScene, _depthImage, _sensorOffset, false);
      _aligner->setReferenceFrame(_subScene);
    }
    else {
      _aligner->setReferenceFrame(reference);
    }
    //    _aligner->setReferenceFrame(reference);
    // Align
    _aligner->clearPriors();
    _aligner->setCurrentFrame(current);
    _aligner->setInitialGuess(initialGuess);
    _aligner->setSensorOffset(_sensorOffset);
    if(hasOdometry) {
      _aligner->addRelativePrior(odometryMean, odometryInfo);
    }
    if(hasImu) {
      _aligner->addAbsolutePrior(reference->globalTransform(), imuMean, imuInfo);
    }
    _aligner->align();
    
    Eigen::Isometry3f localTransformation = _aligner->T();
    if(_aligner->outerIterations() != 0 && (_aligner->inliers() < _minNumInliers || _aligner->error() / _aligner->inliers() > _minError) ) {
      cerr << "ALIGNER FAILURE!!!!!!!!!!!!!!!" << endl;
      cerr << "inliers/minimum number of inliers: " << _aligner->inliers() << " / " << _minNumInliers << endl;
      if(_aligner->inliers() != 0)
	cerr << "error: " << _aligner->error() / _aligner->inliers() << " / " << _minError << endl;
      else
	cerr << "error: " << std::numeric_limits<float>::max() << " / " << _minError << endl;
      localTransformation = initialGuess;
    }
  
    Isometry3f globalT = reference->globalTransform() * localTransformation;
    globalT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    
    // Recondition the rotation to prevent roundoff to accumulate
    Eigen::Matrix3f R = globalT.linear();
    Eigen::Matrix3f E = R.transpose() * R;
    E.diagonal().array() -= 1;
    globalT.linear() -= 0.5 * R * E;

    // Update global and local transforms for the current frame
    current->globalTransform() = globalT;
    current->previousFrameTransform() = localTransformation;

    // Update the g2o graph vertex with the new transform
    Eigen::Isometry3d newEstimate;
    for(int r = 0; r < 3; r++) {
      for(int c = 0; c < 4; c++) {
	newEstimate(r, c) = reference->globalTransform()(r, c);
      }
    }
    newEstimate.matrix().row(3) << 0.0d, 0.0d, 0.0d, 1.0d;
    reference->vertex()->setEstimate(newEstimate);

    if(_aligner->outerIterations() != 0) {
      cout << "Initial guess: " << t2v(initialGuess).transpose() << endl;
      cout << "Local transformation: " << t2v(_aligner->T()).transpose() << endl;
      cout << "Global transformation: " << t2v(globalT).transpose() << endl;
    }

    if(_pwnSaving) {
      Eigen::Isometry3f motionFromFirstFrame = _initialScenePose.inverse() * globalT;
      Eigen::AngleAxisf rotationFromFirstFrame(motionFromFirstFrame.linear());
      if(fabs(rotationFromFirstFrame.angle()) > _chunkAngle || 
	 motionFromFirstFrame.translation().norm() > _chunkDistance) {
	pwn::Point bcenter;
	for (size_t i=0; i<_scene->points().size(); i++){
	  bcenter+=_scene->points().at(i);
	}
	bcenter*=1./_scene->points().size();
	Eigen::Vector3d barycenter(bcenter.x(), bcenter.y(), bcenter.z());
      
	VertexSE3* bestVertex = 0;
	VertexSE3* origin = 0;
	double bestDistance = std::numeric_limits<double>::max();

	for (size_t i=0; i<_sceneVerteces.size(); i++){
	  VertexSE3* currentVertex = _sceneVerteces.at(i);
	  if (! origin) {
	    bestVertex=currentVertex;
	    origin=currentVertex;
	    continue;
	  }
	  Eigen::Isometry3d pLocal=origin->estimate().inverse()*currentVertex->estimate();
	  double currentDistance = (pLocal.translation()-barycenter).squaredNorm();
	  if (bestDistance>currentDistance){
	    bestVertex=currentVertex;
	    bestDistance = currentDistance;
	  }
	}

	assert(bestVertex && "Didn't found a best vertex");
      
	char buff[1024];
	bestVertex = _sceneVerteces.back();
	//sprintf(buff, "out-%05d.pwn", _sceneVerteces.front()->id());
	sprintf(buff, "out-%05d.pwn", bestVertex->id());
	
	Eigen::Isometry3f middleEstimate;
	g2o::VertexSE3 *middleVertex = bestVertex;
	for(int r = 0; r < 3; r++) {
	  for(int c = 0; c < 4; c++) {
	    middleEstimate(r, c) = middleVertex->estimate()(r, c);
	  }
	}
	middleEstimate.matrix().row(3) << 0.0d, 0.0d, 0.0d, 1.0d;
	_scene->transformInPlace(middleEstimate.inverse() * _initialScenePose);
	
	// Voxelize data and recompute stats
	_voxelCalculator->compute(*_scene, 0.01f);

	PWNData *pwnData = new PWNData(_scene);
	pwnData->setFilename(buff);
	pwnData->setOriginPose(middleEstimate);
	pwnData->writeOut();
	cerr << "Saved pwn cloud " << buff << endl;
	// This will also delete the _scene frame
	pwnData->release();
	middleVertex->addUserData(pwnData);
	pwnData->setDataContainer(middleVertex);
	_scene = new Frame();
	_initialScenePose = globalT;
	_sceneVerteces.clear();
      }
    }

    return true;
  }

#ifdef _PWN_USE_TRAVERSABILITY_
  bool PWNMapperController::computeTraversability() { 
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

  void PWNMapperController::updateProjector() {
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

    cerr << "updateProjector" << _projector->cameraMatrix() << endl;
  }
}
