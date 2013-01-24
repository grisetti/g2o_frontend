/*
 * lineEx_viewer.cpp
 *
 *  Created on: Dic 04, 2012
 *      Author: Martina
 */

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

#include "g2o_frontend/thesis/LaserRobotData.h"
#include "line_extraction2d.h"


#include <qapplication.h>
#include <qobject.h>
#include "viewerGUI.h"
// #include <GL/glut.h>

using namespace std;
using namespace g2o;

//useless
VertexSE2 g;

//to be deleted?
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

int main(int argc, char**argv){
  hasToStop = false;
  string filename;
  string outfilename;
  bool noLoop;
  CommandArgs arg;
  int localMapSize;
  int minFeaturesInCluster;
  int minFrameToFrameInliers;
  int minLandmarkCreationFrames;
  int optimizeEachN;
  int dumpEachN;
  double incrementalGuessMaxFeatureDistance;
  double incrementalRansacInlierThreshold;
  double incrementalDistanceDifferenceThreshold;
  double loopAngularClosureThreshold;
  int    loopGuessMaxFeatureDistance;
  double loopLinearClosureThreshold;
  double loopRansacIdentityMatches;
  double loopRansacInlierThreshold;
  double loopRansacInlierRatio;
  double loopLandmarkMergeDistance;
  int intraFrameConnectivityThreshold;
  int localOptimizeIterations;
  int globalOptimizeIterations;
  int updateViewerEachN;
  float maxRange;
  float minRange;
  bool odometryIsGood;
  int incrementalFeatureTrackingWindow;
  arg.param("o", outfilename, "otest.g2o", "output file name"); 
  arg.param("maxRange", maxRange, 1e3, "maximum range to sense features"); 
  arg.param("minRange", minRange, 0.5, "minimum range to sense features");
  arg.param("minLandmarkCreationFrames", minLandmarkCreationFrames, 2, "minimum range to sense features");
  arg.param("noLoop", noLoop, false, "disable loop closure"); 
  arg.param("odometryIsGood", odometryIsGood, false, "use settings for good odometry"); 
  arg.param("localMapSize", localMapSize, 5, "num of nodes in the path to use in the local map"); 
  arg.param("incrementalFeatureTrackingWindow", incrementalFeatureTrackingWindow, 4, "num of frames to look before to seek for a feature match"); 
  arg.param("incrementalGuessMaxFeatureDistance", incrementalGuessMaxFeatureDistance, 1., "max distance between predicted and projected feature, used to process raw readings"); 
  arg.param("incrementalRansacInlierThreshold", incrementalRansacInlierThreshold, 0.3, "inlier distance in the incremental phase"); 
  arg.param("incrementalDistanceDifferenceThreshold", incrementalDistanceDifferenceThreshold, 0.2, "min distance between an inlier and the second best inlier for the same feature"); 
  
  arg.param("loopLinearClosureThreshold", loopLinearClosureThreshold, 5., "max linear distance between frames to check for loop closure");
  arg.param("loopAngularClosureThreshold", loopAngularClosureThreshold, M_PI, "max angular between frames to check for loop closure");
  arg.param("loopGuessMaxFeatureDistance", loopGuessMaxFeatureDistance, 2., "max distance between predicted and projected feature, used to determine correspondence in loop closing"); 
  arg.param("loopRansacIdentityMatches", loopRansacIdentityMatches, 4, "min number of matches between known landmarks to use for ransac to force the alignment between two local maps"); 
  arg.param("loopRansacInlierThreshold", loopRansacInlierThreshold, .3, "inlier distance threshold for ransac in loop closing"); 
  arg.param("loopRansacInlierRatio", loopRansacInlierRatio, .8, "inlier fraction  for ransac in loop closing"); 
  arg.param("loopLandmarkMergeDistance", loopLandmarkMergeDistance, .1, "distance between two instances of landmarks in the local under which the merging occursx"); 
  
  arg.param("intraFrameConnectivityThreshold", intraFrameConnectivityThreshold, 2, "num of landmarks that should be seen by two frames to consider them neighbors"); 
  arg.param("localOptimizeIterations", localOptimizeIterations, 3, "iterations of the optimizer during the local phase (local maps, incremental matching)"); 
  arg.param("globalOptimizeIterations", globalOptimizeIterations, 10, "iterations of the optimizer when merging landmarks"); 
  arg.param("optimizeEachN", optimizeEachN, 5, "perform a global optimization each <x> frames"); 
  arg.param("dumpEachN", dumpEachN, -1, "saves a dump of the running SLAM problem each <x> iterations"); 
  arg.param("updateViewerEachN", updateViewerEachN, -1, "updates the viewer of the running SLAM problem each <x> iterations"); 

  arg.param("minFeaturesInCluster", minFeaturesInCluster, 10, "min num of features to consider in a cluster "); 
  arg.param("minFrameToFrameInliers", minFrameToFrameInliers, 0, "min num of matching features to do something with the frame"); 
  arg.paramLeftOver("graph-input", filename , "", "graph file which will be processed", true);
  
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
	
	LaserRobotData* ldata = 0;
	LaserRobotData::Vector2fVector pointsOriginal;
	//changing this...
	//LaserDataVector ldvector;
	//int this...
	VertexDataVector vldvector;
	
	Eigen::Isometry2d offset = Eigen::Isometry2d::Identity();
	for (size_t i=0; i<vertexIds.size() && ! hasToStop; i++){
		
    OptimizableGraph::Vertex* _v=graph->vertex(vertexIds[i]);
// 		VertexSE3* v=dynamic_cast<VertexSE3*>(_v);
    VertexSE2* v=dynamic_cast<VertexSE2*>(_v);
		
		
    if (!v)
      continue;
		
		//read laser data from the graph constructed given the graph.g2o as filename
    OptimizableGraph::Data* d = v->userData();	
    while(d){
			ldata = dynamic_cast<LaserRobotData*>(d);
			d=d->next();
      if (ldata) {
				//get laser Parameter
				const Parameter* p = graph->parameters().getParameter(ldata->paramIndex());
				const ParameterSE3Offset* param = dynamic_cast<const ParameterSE3Offset*> (p);
// 				TODO
				offset.setIdentity();//param->offset();
				pointsOriginal = ldata->floatCartesian();
				if (pointsOriginal.size()==0) {
					cerr << "WARNING! No laser ranges detected, the g2o file you are using is wrong" << endl;
					return 0;
				}
				//for generating lines files, changing this...
				//ldvector.push_back(make_pair(ldata,pointsOriginal));
				//in this
        vldvector.push_back(make_pair(v,pointsOriginal));				
//			cout << "LaserDataVector size is "<< ldvector.size() << "\tthe last reading has " << ldvector[i].second.size() << " points." << endl;
      }
		}
	}
	cout << "End of file!" << endl;

#if 0
			ofstream osp("points.dat");
			for (size_t i =0; i<pointsOriginal.size(); i++){
					osp << pointsOriginal[i].transpose() << endl;
			}
			osp.flush();
#endif

	QApplication app(argc, argv);
// 	glutInit(&argc, argv);
	//changing this...
// 	ViewerGUI *dialog = new ViewerGUI(&ldvector);
// 	dialog->viewer->setDataPointer(&(ldvector[0].second));
	//in this..
	ViewerGUI *dialog = new ViewerGUI(&vldvector, offset);
	dialog->viewer->setDataPointer(&(vldvector[0].second));
	dialog->show();
	return app.exec();
}
