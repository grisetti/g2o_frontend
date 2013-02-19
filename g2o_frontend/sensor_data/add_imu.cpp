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

#include "laser_robot_data.h"
#include "rgbd_data.h"
#include "imu_data.h"

using namespace std;
using namespace g2o;

volatile bool hasToStop;
void sigquit_handler(int sig)
{
  if (sig == SIGINT) {
    hasToStop = 1;
    static int cnt = 0;
    if (cnt++ == 2) {
      cerr << __PRETTY_FUNCTION__ << " forcing exit" << endl;
      exit(1);
    }
  }
}

// these are to force the linking in case
// ld wants to be picky.

VertexSE3* v=new VertexSE3;
EdgeSE3* e = new EdgeSE3;
LaserRobotData* lrd = new LaserRobotData;
ParameterCamera* pc = new ParameterCamera;
ParameterSE3Offset* po = new ParameterSE3Offset;
RGBDData* rgbd=new RGBDData;
ImuData* imu = new ImuData;

int main(int argc, char**argv){
  hasToStop = false;
  string filename;
  string outfilename;
  CommandArgs arg;
  arg.paramLeftOver("graph-input", filename , "", "graph file which will be processed", true);
  arg.paramLeftOver("graph-output", outfilename , "", "output graph file", true);
  
  arg.parseArgs(argc, argv);
  
  // graph construction
  typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
  OptimizationAlgorithmGaussNewton* solverGauss   = new OptimizationAlgorithmGaussNewton(blockSolver);
  SparseOptimizer * graph = new SparseOptimizer();
  graph->setAlgorithm(solverGauss);
  graph->load(filename.c_str());
  
  // sort the vertices based on the id
  std::vector<int> vertexIds(graph->vertices().size());
  int k=0;
  for (OptimizableGraph::VertexIDMap::iterator it=graph->vertices().begin(); it!= graph->vertices().end(); it ++){
    vertexIds[k++] = (it->first);
  }

  std::sort(vertexIds.begin(), vertexIds.end());
  
  Factory* factory = Factory::instance();
  bool firstVertex = true;
  for (size_t i=0; i<vertexIds.size() && ! hasToStop; i++){
    OptimizableGraph::Vertex* v=graph->vertex(vertexIds[i]);
    cerr << "vertex: " << v->id() << " type:" << factory->tag(v) << endl;
    OptimizableGraph::Data* d = v->userData();
    k = 0;
    while(d){
      if (d) {
	cerr << "\t payload: " << factory->tag(d) << endl;
	const ImuData *imuData = dynamic_cast<const ImuData*>(d);
	VertexSE3* v3=dynamic_cast<VertexSE3*>(v);
	if (imuData && v3){
	  EdgeSE3Prior* imuEdge=new EdgeSE3Prior;
	  imuEdge->setVertex(0,v);
	  imuEdge->setParameterId(0,imuData->paramIndex());
	  Eigen::Isometry3d measurement;
	  measurement.setIdentity();
	  Eigen::Quaterniond q = imuData->getOrientation();
	  if (q.w()<0){
	    q.x()=-q.x();
	    q.y()=-q.y();
	    q.z()=-q.z();
	    q.w()=-q.w();
	  }
	  measurement.linear()=q.toRotationMatrix();
	  measurement.translation()=v3->estimate().translation();
	  imuEdge->setMeasurement(measurement);
	  Slam3dAddons::Matrix6d info;
	  info.setZero();
	  if (firstVertex){
	    info.setIdentity();
	    firstVertex = false;
	  } else {
	    info.block<3,3>(3,3)= Eigen::Matrix3d::Identity() * 1000.0;
	  }
	  imuEdge->setInformation(info);
	  bool result = graph->addEdge(imuEdge);
	  cerr << "edge added to vertex " << v->id() << " parameter: " << imuData->paramIndex() << endl;
	}
	// write here something for doing something with the data you just pulled
	k++;
      }
      d=d->next();
    }
  }
  if (outfilename != ""){
    cerr << "writing out [" << outfilename << "]"<< endl;
    graph->save(outfilename.c_str());
  }
}
