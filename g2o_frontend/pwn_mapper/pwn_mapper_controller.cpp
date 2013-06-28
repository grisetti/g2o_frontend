#include "pwn_mapper_controller.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam3d/types_slam3d.h"

#include <unistd.h>
#include <iostream> 

using namespace std;
using namespace g2o;

namespace pwn {

void PWNMapperController::init(OptimizableGraph *graph_) {
  graph = graph_;
  imageRows = 0;
  imageCols = 0;

  al_minNumInliers = 10000;
  al_minError = 10.0f;

  ng_worldRadius = 0.1f;
  ng_minImageRadius = 10;
  ng_curvatureThreshold = 1.0f;
  al_innerIterations = 1;
  al_outerIterations = 10;
  if_curvatureThreshold = 0.1f;
  reduction = 2;

  _maxDequeSize = 20;

  numProjectors = 4;
  projector = new PinholePointProjector();
  multiProjector = new MultiPointProjector();
  for(int i = 0; i < numProjectors; i++) {
    Isometry3f sensorOffset = Isometry3f::Identity();
    sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;    
    multiProjector->addPointProjector(projector, sensorOffset, 0, 0);
  }


  statsCalculator = new StatsCalculator();

  pointInformationMatrixCalculator = new PointInformationMatrixCalculator();
  normalInformationMatrixCalculator = new NormalInformationMatrixCalculator();
  converter = new DepthImageConverter(projector, statsCalculator, 
				      pointInformationMatrixCalculator, normalInformationMatrixCalculator);
  multiConverter = new DepthImageConverter(multiProjector, statsCalculator, 
				      pointInformationMatrixCalculator, normalInformationMatrixCalculator);

#ifdef _PWN_USE_TRAVERSABILITY_
  traversabilityAnalyzer = new TraversabilityAnalyzer(30, 0.04, 0.2, 1);
#endif //_PWN_USE_TRAVERSABILITY_

  correspondenceFinder = new CorrespondenceFinder();
  linearizer = new Linearizer() ;
  aligner = new Aligner();
    
  aligner->setProjector(multiProjector);
  aligner->setLinearizer(linearizer);
  linearizer->setAligner(aligner);
  aligner->setCorrespondenceFinder(correspondenceFinder);
 
  statsCalculator->setWorldRadius(ng_worldRadius);
  statsCalculator->setMinPoints(ng_minImageRadius);
  aligner->setInnerIterations(al_innerIterations);
  aligner->setOuterIterations(al_outerIterations);
  converter->_curvatureThreshold = ng_curvatureThreshold;
  multiConverter->_curvatureThreshold = ng_curvatureThreshold;
  pointInformationMatrixCalculator->setCurvatureThreshold(if_curvatureThreshold);
  normalInformationMatrixCalculator->setCurvatureThreshold(if_curvatureThreshold);
  
  merger = new Merger();
  merger->setDepthImageConverter(multiConverter);

  _chunkStep = 1000000;
  _chunkAngle = M_PI/4;
  _chunkDistance = 0.5f;

  counter = 0;
  initialPose = Isometry3f::Identity();
  initialPose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
}

void PWNMapperController::clear() {
  while(!_framesDeque.empty()) {
    delete _framesDeque.front();
    _framesDeque.pop_front();
  }
  globalT = Isometry3f::Identity();
}
  
bool extractRelativePrior(Eigen::Isometry3f &priorMean, 
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

bool extractAbsolutePrior(Eigen::Isometry3f &priorMean, 
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

bool PWNMapperController::alignIncrementally(){
  if (_framesDeque.size() < 2)
    return false;

  G2OFrame *current = _framesDeque.back();
  G2OFrame *reference = current->previousFrame();

  cerr << "********************** Aligning vertex " << current->vertex()->id() << " **********************" << endl;
  
  if(mergedClouds.points().size() == 0) 
    mergedClouds.add(*reference, Isometry3f::Identity());

  Eigen::Isometry3f initialGuess;
  // computing initial guess based on the frame positions, just for convenience
  Eigen::Isometry3d delta = reference->vertex()->estimate().inverse()*current->vertex()->estimate();
  for(int c = 0; c < 4; c++)
    for(int r = 0; r < 3; r++)
      initialGuess.matrix()(r, c) = delta.matrix()(r, c);

  Eigen::Isometry3f odometryMean;
  Matrix6f odometryInfo;
  bool hasOdometry = extractRelativePrior(odometryMean, odometryInfo, reference, current);
  if (hasOdometry)
    initialGuess=odometryMean;
  else { //force a prior
    hasOdometry = true;
    odometryMean = initialGuess;
    odometryInfo.setIdentity();
    odometryInfo.block<3,3>(0, 0) *= 100;
  }
  Eigen::Isometry3f imuMean;
  Matrix6f imuInfo;
  bool hasImu = extractAbsolutePrior(imuMean, imuInfo, current);

  initialGuess.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

  projector->setCameraMatrix(current->cameraMatrix());
      
  aligner->clearPriors();
  aligner->setOuterIterations(al_outerIterations);
  
  if(ii.cols() != imageCols || ii.rows() != imageRows) 
    ii.resize(imageRows, imageCols);
  multiProjector->setTransform(initialPose.inverse()*reference->globalTransform());
  multiProjector->project(ii, di, mergedClouds.points());
  multiConverter->compute(subScene, di, Isometry3f::Identity(), true);
  aligner->setReferenceFrame(&subScene);

  aligner->setCurrentFrame(current);
  aligner->setInitialGuess(initialGuess);
  aligner->setSensorOffset(Isometry3f::Identity());
  if(hasOdometry)
    aligner->addRelativePrior(odometryMean, odometryInfo);
  if(hasImu)
    aligner->addAbsolutePrior(initialPose.inverse()*reference->globalTransform(), imuMean, imuInfo);
  aligner->align();
  
  Eigen::Isometry3f localTransformation = aligner->T();
  if(aligner->outerIterations() != 0 && (aligner->inliers() < al_minNumInliers || aligner->error() / aligner->inliers() > al_minError) ) {
    cerr << "ALIGNER FAILURE!!!!!!!!!!!!!!!" << endl;
    cerr << "inliers/minimum number of inliers: " << aligner->inliers() << " / " << al_minNumInliers << endl;
    cerr << "error/minimum error: " << aligner->error() / aligner->inliers() << " / " << al_minError << endl;
    localTransformation = initialGuess;
  }
  

  globalT = reference->globalTransform()*localTransformation;
  // recondition the rotation to prevent roundoff to accumulate
  
  globalT = v2t(t2v(globalT));
  
  if(aligner->outerIterations() != 0) {
    cout << "Initial guess: " << t2v(initialGuess).transpose() << endl;
    cout << "Local transformation: " << t2v(aligner->T()).transpose() << endl;
    cout << "Global transformation: " << t2v(globalT).transpose() << endl;
  }

  Eigen::Isometry3f motionFromFirstFrame = initialPose.inverse()*globalT;
  Eigen::AngleAxisf rotationFromFirstFrame(motionFromFirstFrame.linear());
  if(fabs(rotationFromFirstFrame.angle()) > _chunkAngle || motionFromFirstFrame.translation().norm() > _chunkDistance) {
    char buff[1024];
    sprintf(buff, "out-%05d.pwn", counter++);	
    mergedClouds.save(buff, 1, true, initialPose);
    cerr << "SAVED MERGED CLOUD!!!!" << endl;
    mergedClouds.clear();
    initialPose = globalT;
    motionFromFirstFrame = Isometry3f::Identity();
  }

  // Update cloud drawing position.
  current->globalTransform() = globalT;
  current->previousFrameTransform() = localTransformation;
  mergedClouds.add(*current, motionFromFirstFrame);
  merger->merge(&mergedClouds, current->globalTransform());
  return true;
}

bool PWNMapperController::computeTraversability() { 
  G2OFrame *current = _framesDeque.back();
  if(!current)
    return false;
  G2OFrame globalFrame(*current);
  Eigen::Isometry3f globalTransform = current->globalTransform();
  globalTransform.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  globalFrame.transformInPlace(globalTransform);
#ifdef _PWN_USE_TRAVERSABILITY_
  traversabilityAnalyzer->createTraversabilityVector(globalFrame.points(), globalFrame.normals(), globalFrame.traversabilityVector()); 
#endif //_PWN_USE_TRAVERSABILITY_
  return true;
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
  g2o::Parameter *_cameraParam = graph->parameter(paramIndex);
  // attempt a cast to a parameter camera  
  ParameterCamera *cameraParam = dynamic_cast<ParameterCamera*>(_cameraParam);
  if(!cameraParam) {
    cerr << "Could not find a valid camera" << endl;
    return false;
  }
    
  // We got the parameter
  Eigen::Matrix3f cameraMatrix;
  Eigen::Isometry3f sensorOffset;
  cameraMatrix.setZero();
    
  int cmax = 4;
  int rmax = 3;
  for(int c = 0; c < cmax; c++) {
    for(int r = 0; r < rmax; r++) {
      sensorOffset.matrix()(r, c) = cameraParam->offset()(r, c);
      if(c < 3)
	cameraMatrix(r, c) = cameraParam->Kcam()(r, c);
    }
  }
  
  cameraMatrix << 
    525.0f, 0.0f, 319.5f,
    0.0f, 525.0f, 239.5f,
    0.0f, 0.0f, 1.0f;
  
  sensorOffset = Isometry3f::Identity();
  sensorOffset.translation() = Vector3f(0.15f, 0.0f, 0.05f);
  Quaternionf quat = Quaternionf(0.5f, -0.5f, 0.5f, -0.5f);
  sensorOffset.linear() = quat.toRotationMatrix();
  sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

  std::string fname = rgbdData->baseFilename() + "_depth.pgm";
  // UGLY
  DepthImage depthImage;
  if (!depthImage.load(fname.c_str(), true)){
    cerr << "No depth image loaded" << endl;
    return false;
  }
    
  // do the scaling of the camera matrix
  float scale = 1./reduction;
  cameraMatrix *= scale;
  cameraMatrix(2,2) = 1.0;
      
  // do the scaling of the image
  DepthImage scaledDepthImage;
  DepthImage::scale(scaledDepthImage, depthImage, reduction);

  for(int i = 0; i < numProjectors; i++) {
    Isometry3f sensorOffset = Isometry3f::Identity();
    sensorOffset.translation() = Vector3f(0.15f, 0.0f, 0.05f);
    Quaternionf quat = Quaternionf(0.5, -0.5, 0.5, -0.5);
    if(i > 0)
      sensorOffset.linear() =  AngleAxisf(i * M_PI / 2.0f, Vector3f::UnitZ()) * quat.toRotationMatrix();
    else
      sensorOffset.linear() =  quat.toRotationMatrix();
    sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    
    multiProjector->setPointProjector(projector, sensorOffset, scaledDepthImage.rows(), scaledDepthImage.cols(), i);
  }

  // should be done here???
  multiProjector->size(imageRows, imageCols);
  //  imageRows = scaledDepthImage.rows();
  //  imageCols = scaledDepthImage.cols() * numProjectors;
  correspondenceFinder->setSize(imageRows, imageCols);
  merger->setImageSize(imageRows, imageCols);

  // create the frame and set the parameters
  G2OFrame *frame = new G2OFrame(v);
  frame->cameraMatrix() = cameraMatrix;
  frame->sensorOffset() = sensorOffset;
    
  G2OFrame *previousFrame = 0;
  if(!_framesDeque.empty()) 
    previousFrame = _framesDeque.back();
    
  // of it is the first vertex, we put the imu as prior if avalable
  if(!previousFrame) {
    globalT.setIdentity();
    frame->globalTransform().setIdentity();
    frame->previousFrameTransform().setIdentity();
    Eigen::Isometry3f priorMean;
    Matrix6f priorInfo;
    bool hasImu = extractAbsolutePrior(priorMean, priorInfo, frame);
    if (hasImu){
      globalT.linear() = priorMean.linear();
      cerr << "Found an IMU for the first vertex" << endl;
      frame->globalTransform() = globalT;
    }
  }
  frame->setPreviousFrame(previousFrame);

  projector->setCameraMatrix(cameraMatrix);
  converter->compute(*frame, scaledDepthImage, sensorOffset, true);
  _framesDeque.push_back(frame);

  // Keep at max maxDequeSize elements in the queue
  while (_framesDeque.size() > _maxDequeSize) {
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
  g2o::Parameter *_cameraParam = graph->parameter(paramIndex);
  ParameterCamera *cameraParam = dynamic_cast<ParameterCamera*>(_cameraParam);
  if(!cameraParam) {
    cerr << "Could not find a valid camera" << endl;
    return false;
  }
    
  Eigen::Matrix3f cameraMatrix;
  Eigen::Isometry3f sensorOffset;
  
  cameraMatrix << 
    525.0f, 0.0f, 319.5f,
    0.0f, 525.0f, 239.5f,
    0.0f, 0.0f, 1.0f;
  
  sensorOffset = Isometry3f::Identity();
  sensorOffset.translation() = Vector3f(0.15f, 0.0f, 0.05f);
  Quaternionf quat = Quaternionf(0.5f, -0.5f, 0.5f, -0.5f);
  sensorOffset.linear() = quat.toRotationMatrix();
  sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

  std::string fname = rgbdData->baseFilename() + "_depth.pgm";
  DepthImage depthImage;
  if(!depthImage.load(fname.c_str(), true)) {
    cerr << "No depth image loaded." << endl;
    return false;
  }
    
  float scale = 1./reduction;
  cameraMatrix *= scale;
  cameraMatrix(2, 2) = 1.0;
      
  DepthImage scaledDepthImage;
  DepthImage::scale(scaledDepthImage, depthImage, reduction);

  for(int i = 0; i < numProjectors; i++) {
    Isometry3f sensorOffset = Isometry3f::Identity();
    sensorOffset.translation() = Vector3f(0.15f, 0.0f, 0.05f);
    Quaternionf quat = Quaternionf(0.5, -0.5, 0.5, -0.5);
    if(i > 0)
      sensorOffset.linear() =  AngleAxisf(i * M_PI / 2.0f, Vector3f::UnitZ()) * quat.toRotationMatrix();
    else
      sensorOffset.linear() =  quat.toRotationMatrix();
    sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    
    multiProjector->setPointProjector(projector, sensorOffset, scaledDepthImage.rows(), scaledDepthImage.cols(), i);
  }

  // should be done here???
  multiProjector->size(imageRows, imageCols);
  //  imageRows = scaledDepthImage.rows();
  //  imageCols = scaledDepthImage.cols() * numProjectors;

  frame.cameraMatrix() = cameraMatrix;
  frame.sensorOffset() = sensorOffset;
    
  globalT.setIdentity();
  frame.globalTransform().setIdentity();
  frame.previousFrameTransform().setIdentity();
  frame.setPreviousFrame(0);

  projector->setCameraMatrix(cameraMatrix);
  converter->compute(frame, scaledDepthImage, sensorOffset, true);
  
  return true;
}

}
 
