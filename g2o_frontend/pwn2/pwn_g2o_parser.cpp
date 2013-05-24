#include <signal.h>

#include "g2o/stuff/command_args.h"
#include "g2o/stuff/timeutil.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"

#include "g2o_frontend/sensor_data/laser_robot_data.h"
#include "g2o_frontend/sensor_data/rgbd_data.h"
#include "g2o_frontend/sensor_data/imu_data.h"

#include "g2o_frontend/pwn2/frame.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/aligner.h"
#include "g2o_frontend/basemath/bm_se3.h"

#include <fstream>

using namespace std;
using namespace g2o;
using namespace pwn;
volatile bool hasToStop;
void sigquit_handler(int sig)
{
  if(sig == SIGINT)
  {
    hasToStop = 1;
    static int cnt = 0;
    if(cnt++ == 2)
    {
      cerr << __PRETTY_FUNCTION__ << " forcing exit" << endl;
      exit(1);
    }
  }
}

// these are to force the linking in case
// ld wants to be picky.
VertexSE3* v = new VertexSE3;
EdgeSE3* e = new EdgeSE3;
LaserRobotData* lrd = new LaserRobotData;
ParameterCamera* pc = new ParameterCamera;
ParameterSE3Offset* po = new ParameterSE3Offset;
RGBDData* rgbd = new RGBDData;
ImuData* imu = new ImuData;

int main(int argc, char**argv) {
  int al_innerIterations, al_outerIterations;
  PinholePointProjector *projector = new PinholePointProjector();
  StatsFinder *statsFinder = new StatsFinder();
  PointInformationMatrixFinder *pointInformationMatrixFinder = new PointInformationMatrixFinder();
  NormalInformationMatrixFinder *normalInformationMatrixFinder = new NormalInformationMatrixFinder();
  DepthImageConverter depthImageConverter = DepthImageConverter(projector, statsFinder, 
								     pointInformationMatrixFinder, normalInformationMatrixFinder);
  
		


  hasToStop = false;
  string filename;
  string outputFilename;
  
  CommandArgs arg;
  arg.param("al_innerIterations", al_innerIterations, 10, "Specify the inner iterations. [int]");
  arg.param("al_outerIterations", al_outerIterations, 10, "Specify the outer iterations. [int]");
  arg.paramLeftOver("graph-input", filename , "", "graph file which will be processed", true);
  arg.paramLeftOver("graph-output", outputFilename , "", "output graph file", true);
  arg.parseArgs(argc, argv);

  DepthImage depthImage;
  CorrespondenceFinder correspondenceFinder;
  Linearizer linearizer;
  Aligner aligner;
  aligner.setLinearizer(&linearizer);
  linearizer.setAligner(&aligner);
  aligner.setProjector(projector);
  aligner.setCorrespondenceFinder(&correspondenceFinder);
  
  aligner.setOuterIterations(al_outerIterations);
  aligner.setInnerIterations(al_innerIterations);

  // graph construction
  typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
  OptimizationAlgorithmGaussNewton* solverGauss = new OptimizationAlgorithmGaussNewton(blockSolver);
  SparseOptimizer* graph = new SparseOptimizer();
  graph->setAlgorithm(solverGauss);
  graph->load(filename.c_str());
  // sort the vertices based on the id
  vector<int> vertexIds(graph->vertices().size());
  int k=0;
  for (OptimizableGraph::VertexIDMap::iterator it = graph->vertices().begin(); it != graph->vertices().end(); ++it) {
    vertexIds[k++] = (it->first);
  }

  sort(vertexIds.begin(), vertexIds.end());

  Factory* factory = Factory::instance();
  
  size_t maxCount = 20;
  
  Frame *referenceFrame = 0,
        *currentFrame = 0;
  
  Isometry3f trajectory = Isometry3f::Identity();
  
  for(size_t i = 0; i < vertexIds.size() &&  i< maxCount && !hasToStop; ++i) {
    OptimizableGraph::Vertex* _v = graph->vertex(vertexIds[i]);
    g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(_v);
    if (! v)
      continue;
    cerr << "vertex: " << v->id() << " type:" << factory->tag(v) << endl;
    OptimizableGraph::Data* d = v->userData();
    while(d) {
      RGBDData* rgbdData = dynamic_cast<RGBDData*>(d);
      if (!rgbdData){
	d=d->next();
	continue;
      }
      cerr << "got data" << endl;
      // retrieve from the rgb data the index of the parameter
      int paramIndex = rgbdData->paramIndex();
      // retrieve from the graph the parameter given the index  
      g2o::Parameter* _cameraParam = graph->parameter(paramIndex);
      // attempt a cast to a parameter camera  
      ParameterCamera* cameraParam = dynamic_cast<ParameterCamera*>(_cameraParam);
      if (! cameraParam){
	cerr << "shall thou be damned forever" << endl;
	return 0;
      }
      // yayyy we got the parameter
      Eigen::Matrix3f cameraMatrix;
      Eigen::Isometry3f sensorOffset;
      cameraMatrix.setZero();
      
      int cmax = 4;
      int rmax = 3;
      for (int c=0; c<cmax; c++){
	for (int r=0; r<rmax; r++){
	  sensorOffset.matrix()(r,c)= cameraParam->offset()(r, c);
	  if (c<3)
	    cameraMatrix(r,c) = cameraParam->Kcam()(r, c);
	}
      }
      sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
      // set the things we need to set to compute the scene, given the image
      projector->setCameraMatrix(cameraMatrix);

      Frame* frame = new Frame();

      std::string currentFilename = rgbdData->baseFilename() + "_depth.pgm";
      bool loadOk = depthImage.load(currentFilename.c_str());
      if (!loadOk) {
	cerr << "Failure while loading depth image " << currentFilename << " ... , skipping the frame. " << endl;
	delete frame;
      }
      //cerr << "opphsette" << endl;
      //cerr << sensorOffset.matrix() << endl;
      //cerr << "cum-era matrix" << endl;
      //cerr << cameraMatrix << endl;
      //sensorOffset = Eigen::Isometry3f::Identity();
      sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
      depthImageConverter.compute(*frame, depthImage, sensorOffset);      
      currentFrame = frame;

      if (referenceFrame) {
	aligner.correspondenceFinder()->setSize(depthImage.rows(), depthImage.cols());

	aligner.setReferenceFrame(referenceFrame);
	aligner.setCurrentFrame(currentFrame);

	Isometry3f initialGuess = Isometry3f::Identity();
	initialGuess.matrix().row(3) << 0, 0, 0, 1;
	aligner.setInitialGuess(initialGuess);
	aligner.setSensorOffset(sensorOffset);
	
	aligner.align();
	cerr << "inliers: " << aligner.inliers() << " error: " << aligner.error() << endl;
	cerr << endl << aligner.T().matrix() << endl;

	trajectory = aligner.T() * trajectory;
	char buff[1024];
	sprintf(buff, "out-%03d.pwn", (int)i);
	currentFrame->save(buff, trajectory, 1, true);
      }
      
      d = d->next();
    }
    
    if (referenceFrame) {
      delete referenceFrame;
      referenceFrame = 0;
    }
    referenceFrame = currentFrame;
  }
    /*	if(rgbdData && vse3) {
	  
	  if (vOld) {
	    previousT = aligner->T();
	    cerr << "aligning with vertex:" << vOld->id() << endl; 
	    cerr << "robotOffset:" << t2v(vOld->estimate().inverse() * vse3->estimate()).transpose() << endl; 
	  }
	  else {
	    if(pNew) {
	      cameraMatrix(0, 0) = pNew->Kcam()(0, 0);
	      cameraMatrix(1, 1) = pNew->Kcam()(1, 1);
	      cameraMatrix(0, 2) = pNew->Kcam()(0, 2);
	      cameraMatrix(1, 2) = pNew->Kcam()(1, 2);
	      cameraMatrix(2, 2) = pNew->Kcam()(2, 2);
	      offset = pNew->offsetMatrix();
	    }
	    projector->setCameraMatrix(cameraMatrix);
	  }

	  // Invece di frame usa la nuova classe converter di giorgio.
	  Frame* currentFrame = new Frame();
	  std::string currentFilename = rgbdData->baseFilename() + "_depth.pgm";
	  bool loadOk = depthImage->load(currentFilename.c_str());
	  if (!loadOk) {
	    cerr << "Failure while loading depth image " << currentFilename << " ... , skipping the frame. " << endl;
	    delete currentFrame;
	    currentFrame = 0;
	    continue;
	  }

	  depthImageConverter->compute(*currentFrame, *depthImage);
	  cerr << "*******************************************************" << endl;
	  cerr << "currentFilename: [" << currentFilename << "]" << endl;
	  
	  Eigen::Isometry3d currentRobotPose = vse3->estimate();
	  Eigen::Isometry3d currentCameraPose = currentRobotPose*pNew->offset();
	  if (referenceFrame && oldData && vOld ) {
	    Eigen::Isometry3d oldRobotPose = vOld->estimate();
	    g2o::Parameter* _pOld = graph->parameter(oldData->paramIndex());
	    ParameterCamera* pOld = dynamic_cast<ParameterCamera*>(_pOld);
	    //g2o::Parameter* pOffsetOld = graph->parameter(oldData->paramIndex()+10);
	    Eigen::Isometry3d oldCameraPose = oldRobotPose*pOld->offset();
	    Eigen::Isometry3d oldCameraMovement = oldCameraPose.inverse()*currentCameraPose;
	    
	    aligner->setProjector(projector);
	    aligner->setLinearizer(linearizer);
	    linearizer->setAligner(aligner);
	    aligner->setCorrespondenceFinder(correspondenceFinder);
	    
	    aligner->setOuterIterations(10);
	    aligner->setInnerIterations(1);
	    
	    aligner->correspondenceFinder()->setSize(depthImage->rows(), depthImage->cols());

	    aligner->setReferenceFrame(referenceFrame);
	    aligner->setCurrentFrame(currentFrame);
	    
	    Isometry3f initialGuess(oldCameraMovement);
	    Isometry3f sensorOffset = Isometry3f::Identity();
	    aligner->setInitialGuess(initialGuess);
	    aligner->setSensorOffset(sensorOffset);
	    
	    Matrix6f priorInformationMatrix = Matrix6f::Identity();
	    aligner->addPrior(initialGuess, priorInformationMatrix);

	    aligner->align();
	    
	    cout << "Inliers: " << aligner->inliers() << endl;
	    cout << "Final transformation: " << endl << aligner->T().matrix() << endl;

	    
	  }
	  
	  if (referenceFrame) {
	    char buf[1024];
	    sprintf(buf, "%s%05d.pwn", outputFilename.c_str(), counter);	
	    referenceFrame->transformInPlace(previousT);
	    referenceFrame->save(buf, 1, true);
	    delete referenceFrame;
	    referenceFrame = 0;
	  }
	  referenceFrame = currentFrame;
	  vOld = vse3;
	  oldData = rgbdData;
	}
	
	k++;
      }
      d = d->next();
      counter++;
    }
  }
    */
}
