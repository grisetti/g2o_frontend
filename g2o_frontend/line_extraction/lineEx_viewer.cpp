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

#include <qapplication.h>
#include "opencv2/opencv.hpp"
#include <qobject.h>
#include "viewerGUI.h"

using namespace std;
using namespace g2o;
using namespace cv;

//fro Images
#define fx_d 5.9421434211923247e+02
#define fy_d 5.9104053696870778e+02
#define cx_d 3.3930780975300314e+02
#define cy_d 2.4273913761751615e+02
#define k1_d -2.6386489753128833e-01
#define k2_d 9.9966832163729757e-01
#define p1_d -7.6275862143610667e-04
#define p2_d 5.0350940090814270e-03
#define k3_d -1.3053628089976321e+00

vector<Vec3f> cloud;
vector<Vec3f> cloudOriginal;
Mat tmp;
int allPoints;
int skippedPoints;

//for laser points data
LaserRobotData::Point2DVector pointsLine;
LaserRobotData::Point2DVector pointsOriginal;

// void dumpEdges(ostream& os, BaseFrameSet& fset, 
// 	       bool writeOdometry = true, 
// 	       bool writeLandmarks = true, 
// 	       bool writeIntraFrame = false) {
// 
//    GraphItemSelector graphSelector;
//   graphSelector.compute(fset);
//   HyperGraph::EdgeSet& eset=graphSelector.selectedEdges();
//   OptimizableGraph::VertexSet& vset=graphSelector.selectedVertices();
//   
//   os << "set size ratio -1" << endl;
//   os << "plot ";
//   if (writeOdometry) {
//     os << "'-' w l lw 0.5, ";
//   }
//   if (writeIntraFrame) {
//     os << "'-' w l lw 0.5, ";
//   }
//   if (writeLandmarks) {
//     os << "'-' w p ps 0.5,";
//   }
//   os << "'-' w p ps 1.5" << endl;
// 
//   if (writeOdometry){
//     for (HyperGraph::EdgeSet::iterator it = eset.begin(); it!=eset.end(); it++){
//       const EdgeSE2* e = dynamic_cast<const EdgeSE2*>(*it);
//       if (! e)
// 	continue;
//       const VertexSE2* v1 = dynamic_cast<const VertexSE2*>(e->vertices()[0]);
//       const VertexSE2* v2 = dynamic_cast<const VertexSE2*>(e->vertices()[1]);
//       os << v1->estimate().translation().x() << " " << v1->estimate().translation().y() << endl;
//       os << v2->estimate().translation().x() << " " << v2->estimate().translation().y() << endl;
//       os << endl;
//     }
//     os << "e" << endl;
//   }
// 
//   if (writeIntraFrame) {
//     for (BaseFrameSet::iterator it = fset.begin(); it!=fset.end(); it++){
//       BaseFrame* f = *it;
//       const VertexSE2* v1 = f->vertex<const VertexSE2*>();
//       if (!v1)
// 	continue;
//       for (BaseFrameSet::iterator iit = f->neighbors().begin(); iit!=f->neighbors().end(); iit++){
// 	BaseFrame* f2 = *iit;
// 	if (! fset.count(f2))
// 	  continue;
// 	const VertexSE2* v2 = f2->vertex<const VertexSE2*>();
// 	if (! v2)
// 	  continue;
// 	os << v1->estimate().translation().x() << " " << v1->estimate().translation().y() << " " << v1->estimate().rotation().angle() << endl;
// 	os << v2->estimate().translation().x() << " " << v2->estimate().translation().y() << " " << v2->estimate().rotation().angle() << endl;
// 	os << endl;
//       }
//     }
//     os << "e" << endl;
//   }
// 
//   // write the landmarks
//   if (writeLandmarks) {
//     for (OptimizableGraph::VertexSet::iterator it = vset.begin(); it!=vset.end(); it++){
//       const VertexPointXY* v = dynamic_cast<const VertexPointXY*>(*it);
//       if (! v)
// 	continue;
//       os << v->estimate().x() << " " << v->estimate().y() <<  endl;
//     }
//     os << "e" << endl;
//   }
//   
//   for (OptimizableGraph::VertexSet::iterator it = vset.begin(); it!=vset.end(); it++){
//     const VertexSE2* v = dynamic_cast<const VertexSE2*>(*it);
//     if (! v)
//       continue;
//     os << v->estimate().translation().x() << " " << v->estimate().translation().y() << " " << v->estimate().rotation().angle() << endl;
//   }
//   os << "e" << endl;
//   os << endl;
// }


// void dumpEdges(ostream& os, MapperState* mapperState) {
//   GraphItemSelector graphSelector;
//   BaseFrameSet fset;
//   for (VertexFrameMap::iterator it = mapperState->frames().begin(); it!=mapperState->frames().end(); it++){
//     fset.insert(it->second);
//   }
//   dumpEdges(os, fset, true, true, false);
// }


// void saveThings(string filename , MapperState* mapperState, int seqNum = -1)  {
//   if (seqNum>-1) {
//     string baseFilename = filename.substr(0,filename.find_last_of("."));
//     char seqNumStr[10];
//     sprintf(seqNumStr, "%04d", seqNum);
//     filename = baseFilename + "-" + seqNumStr + ".g2o";
//   }
// 
//   OptimizableGraph::VertexSet vset;
//   HyperGraph::EdgeSet eset;
//   GraphItemSelector graphSelector;
//   BaseFrameSet fset;
//   for (VertexFrameMap::iterator it = mapperState->frames().begin(); it!=mapperState->frames().end(); it++){
//     fset.insert(it->second);
//   }
//   graphSelector.compute(fset);
//   eset=graphSelector.selectedEdges();
//   vset=graphSelector.selectedVertices();
// 
//   ofstream os(filename.c_str());
//   mapperState->graph()->saveSubset(os, eset);
// }


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

//for image
void reproject(Mat &image)
{
	int rows=image.rows;
	int cols=image.cols;
	unsigned short * dptra=tmp.ptr<unsigned short>(0);
	for(int i = 0; i < rows; i++)
	{

		for(int j = 0; j < cols; j++)
		{
			unsigned short d = *dptra;
			if(d != 0)
			{
				//float x=(j-cx_d)*d/fx_d;
				//float y=(i-cy_d)*d/fy_d;
				//float z=d;
				Vec3f p;
				p[0]=(double)((double)j-(double)cx_d)*(double)d/(double)fx_d;
				p[1]=(double)((double)i-(double)cy_d)*(double)d/(double)fy_d;
				p[2]=(double)d;
				
				cloud.push_back(p);
				
				cloudOriginal.push_back(p);
			}
			dptra++;
		}
	}
	cout <<" Reprojection done!"<<endl;
}


//creating a cloud from laser data
void cloudPopulation(LaserRobotData* ldata)
{
	LaserRobotData::Point2DVector points = ldata->cartesian();
	cout << "points: " << points[0].x() << ", " <<points[0].y() << endl;
	Vec3f v;
	
	cout << "cloud prima" << cloud.size() << endl;
	for (size_t i = 0; i < points.size(); i++)
	{
		v[0] = (float)points[i].x();
		v[1] = (float)points[i].y();
		v[2] = 0.f;
		
		cloud.push_back(v);
		cloudOriginal.push_back(v);
	}
	cout << "cloud fine" << cloud.size() << endl;
	cout <<" Population done!"<<endl;
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
	
	LaserRobotData* ldata;
	for (size_t i=0; i<vertexIds.size() && ! hasToStop; i++){
    OptimizableGraph::Vertex* _v=graph->vertex(vertexIds[i]);
    VertexSE3* v=dynamic_cast<VertexSE3*>(_v);
    if (!v)
      continue;
    
		//read laser data from the graph constructed given the graph.g2o as filename
    OptimizableGraph::Data* d = v->userData();
    k = 0;
		
    while(d){
			
			ldata = dynamic_cast<LaserRobotData*>(d);
			d=d->next();
// 			const Parameter* p = graph->parameters().getParameter(ldata->paramIndex());
// 			const ParameterSE3Offset* param = dynamic_cast<const ParameterSE3Offset*> (p);
// 			const Eigen::Isometry3d& offset = param->offset();
// 			glMultMatrixd(offset.data());
      if (ldata && k==0) {
				//cout <<" Laser data read: "<< ldata->ranges().front() << endl;
				//cloudPopulation(ldata);
				pointsOriginal = ldata->cartesian();
				pointsLine = ldata->cartesian();
				//cout <<" Cartesian Laser data read: "<< pointsOriginal[0].x() << ", " << pointsOriginal[0].y() << endl;
      }
			k++; //cout << k << endl;
			
		}
	}
	cout << "File ended!" << endl;

	QApplication app(argc, argv);
	ViewerGUI *dialog = new ViewerGUI(ldata,&pointsLine, &pointsOriginal);
	dialog->viewer->setDataPointer(&pointsLine);
	dialog->show();
	return app.exec();
}
