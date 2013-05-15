#include <signal.h>

#include "g2o/stuff/macros.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/timeutil.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/types/slam2d_addons/types_slam2d_addons.h"

#include "g2o_frontend/pwn2/frame.h"

#include "g2o_frontend/sensor_data/laser_robot_data.h"
#include "g2o_frontend/sensor_data/rgbd_data.h"
#include "g2o_frontend/sensor_data/imu_data.h"

#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/aligner.h"
#include "g2o_frontend/basemath/bm_se3.h"

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
  PinholePointProjector *projector = new PinholePointProjector();
  StatsFinder *statsFinder = new StatsFinder();
  PointInformationMatrixFinder *pointInformationMatrixFinder = new PointInformationMatrixFinder();
  NormalInformationMatrixFinder *normalInformationMatrixFinder = new NormalInformationMatrixFinder();
  DepthImageConverter *depthImageConverter = new DepthImageConverter(projector, statsFinder, 
								     pointInformationMatrixFinder, normalInformationMatrixFinder);
  DepthImage *depthImage = new DepthImage();
  CorrespondenceFinder *correspondenceFinder = new CorrespondenceFinder();
  Linearizer *linearizer = new Linearizer();
  Aligner *aligner = new Aligner;

  hasToStop = false;
  string filename;
  string outputFilename;
  CommandArgs arg;
  arg.paramLeftOver("graph-input", filename , "", "graph file which will be processed", true);
  arg.paramLeftOver("graph-output", outputFilename , "", "output graph file", true);
  arg.parseArgs(argc, argv);

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

  int counter = 0;

  Factory* factory = Factory::instance();
  VertexSE3* vOld = 0;
  RGBDData*  oldData = 0;
  Frame* referenceFrame = 0;
  Matrix3f cameraMatrix = Matrix3f::Identity();

  for(size_t i = 0; i < vertexIds.size() && !hasToStop; ++i) {
    OptimizableGraph::Vertex* v = graph->vertex(vertexIds[i]);
    cerr << "vertex: " << v->id() << " type:" << factory->tag(v) << endl;
    OptimizableGraph::Data* d = v->userData();
    k = 0;
    while(d) {
      if(counter >= 100)
	break;
      if(d) {
	cerr << "\t payload: " << factory->tag(d) << endl;
	RGBDData* rgbdData = dynamic_cast<RGBDData*>(d);
	VertexSE3* vse3 = dynamic_cast<VertexSE3*>(v);
	
	if(rgbdData && vse3) {
	    cerr << "rgbd_data found in vertex:" << vse3->id() << endl;
	    g2o::Parameter* _pNew = graph->parameter(rgbdData->paramIndex());
	    ParameterCamera* pNew = dynamic_cast<ParameterCamera*>(_pNew);
	    ParameterSE3Offset* pOffsetNew = dynamic_cast<ParameterSE3Offset*>(graph->parameter(rgbdData->paramIndex()+10));
	  if (vOld) {
	    cerr << "aligning with vertex:" << vOld->id() << endl; 
	    cerr << "robotOffset:" << t2v(vOld->estimate().inverse() * vse3->estimate()).transpose() << endl; 
	  }
	  else {
	    cameraMatrix(0, 0) = pNew->Kcam()(0, 0);
	    cameraMatrix(1, 1) = pNew->Kcam()(1, 1);
	    cameraMatrix(0, 2) = pNew->Kcam()(0, 2);
	    cameraMatrix(1, 2) = pNew->Kcam()(1, 2);
	    cameraMatrix(2, 2) = pNew->Kcam()(2, 2);
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
	  
	  if (!pOffsetNew) {
	    pOffsetNew = new ParameterSE3Offset;
	    pOffsetNew->setOffset(pNew->offset());
	    pOffsetNew->setId(rgbdData->paramIndex()+10);
	    graph->addParameter(pOffsetNew);
	  } 
	  
	  Eigen::Isometry3d currentRobotPose = vse3->estimate();
	  Eigen::Isometry3d currentCameraPose = currentRobotPose*pNew->offset();
	  if (referenceFrame && oldData && vOld ) {
	    Eigen::Isometry3d oldRobotPose = vOld->estimate();
	    g2o::Parameter* _pOld = graph->parameter(oldData->paramIndex());
	    ParameterCamera* pOld = dynamic_cast<ParameterCamera*>(_pOld);
	    g2o::Parameter* pOffsetOld = graph->parameter(oldData->paramIndex()+10);
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

	    EdgeSE3Offset* eNew = new EdgeSE3Offset;
	    Vector6f _measNew = t2v(initialGuess);
	    Vector6d measNew;
	    Matrix6d omegaNew;
	    for (int i=0; i<6; i++){
	      measNew(i) = double(_measNew(i));
	      for (int j=0; j<6; j++){
		omegaNew(i, j) = double(priorInformationMatrix(i,j));
	      }
	    }
	    eNew->setVertex(0, vOld);
	    eNew->setParameterId(0, pOffsetOld->id());
	    eNew->setVertex(1, vse3);
	    eNew->setParameterId(1, pOffsetNew->id());
	    eNew->setMeasurement(v2t(measNew));
	    eNew->setInformation(omegaNew);
	    graph->addEdge(eNew);
	  }
	  
	  if (referenceFrame) {
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
  
  if(outputFilename != "") {
    graph->save(outputFilename.c_str());
    cout << "Graph saved" << endl;
  }
  else {
    cout << "Output filename not provided" << endl;
  }
}
