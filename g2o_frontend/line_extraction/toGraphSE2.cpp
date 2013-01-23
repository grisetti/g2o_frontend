/*
 * line_extraction2d.cpp
 *
 *  Created on: Jan 22, 2013
 *      Author: Martina
 */

#include "g2o/stuff/macros.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/timeutil.h"

#include <g2o/core/hyper_graph.h>
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/types/slam2d/se2.h"
#include "g2o_frontend/thesis/SensorData.h"
#include "g2o_frontend/thesis/LaserRobotData.h"
#include "g2o_frontend/thesis/RGBDData.h"


using namespace std;
using namespace g2o;

//fittizio e inutile
 	LaserRobotData l;
	
typedef std::pair<g2o::VertexSE3*, g2o::VertexSE2*> Vertex3to2;
typedef std::vector<Vertex3to2> v3Mapv2;

SE2 fromSE3(const Eigen::Isometry3d& iso){
	Eigen::AngleAxisd aa(iso.linear());
	return SE2(iso.translation().x(), iso.translation().y(), aa.angle());
}

int main(int argc, char**argv){
	
  string filename;	
  string outfilename;
	std::ofstream ofG2O(&outfilename[0]);
	CommandArgs arg;
	arg.param("o", outfilename, "otest.g2o", "output file name");
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
	SparseOptimizer * graphSE2 = new SparseOptimizer();
  graph->setAlgorithm(solverGauss);
	graphSE2->setAlgorithm(solverGauss);
  graph->load(filename.c_str());
	
	// sort the vertices based on the id
  std::vector<int> vertexIds(graph->vertices().size());
  int k=0;
  for (OptimizableGraph::VertexIDMap::iterator it=graph->vertices().begin(); it!= graph->vertices().end(); it ++){
    vertexIds[k++] = (it->first);
	}
  std::sort(vertexIds.begin(), vertexIds.end());
	
	v3Mapv2 vertexVector;
	OptimizableGraph::Data* d = 0;
	LaserRobotData* data = 0;
	Sensor* sensor = 0;
// 	OptimizableGraph* graphSE2 = new OptimizableGraph();
	for (size_t i = 0; i<vertexIds.size(); i++)
	{
    OptimizableGraph::Vertex* _v = graph->vertex(vertexIds[i]);
    VertexSE3* v3 = dynamic_cast<VertexSE3*>(_v);
    if (!v3)
      continue;		
		
		VertexSE2* v2 = new VertexSE2();
		v2->setEstimate(fromSE3(v3->estimate()));
		v2->setId(v3->id());
		d = v3->userData();		
	  
		while(d) {	
			data = dynamic_cast<LaserRobotData*>(d);
			d=d->next();
			if(data)
			{
				cout << "i: " << i << endl;
				v2->addUserData(data);
				//adding sensor parameter
				Parameter* p = graph->parameters().getParameter(data->paramIndex());
				if (! graphSE2->parameters().getParameter(data->paramIndex())){
					ParameterSE3Offset* parameter = dynamic_cast<ParameterSE3Offset*> (p);
					graphSE2->parameters().addParameter(parameter);
					graphSE2->saveParameter(ofG2O, parameter);
				}
				graphSE2->addVertex(v2);
				graphSE2->saveVertex(ofG2O, v2);		
				//adding sensor parameter, using SensorData, not working
// 				sensor = data->getSensor();
// 				assert (!sensor && "!");
// 				cout << "pippo" << endl;		
// 				Parameter* parameter = sensor->getParameter();
// 				assert (!parameter && "!");
// 				
// 				if (! graphSE2->parameters().getParameter(parameter->id())){
// 					cout << "pippo3" << endl; 
// 					graphSE2->parameters().addParameter(parameter);
// 					cout << "pippo4" << endl;
// 					graphSE2->saveParameter(ofG2O, parameter);
// 				}
			}
		}
		vertexVector.push_back(make_pair(v3, v2));	
	}
		
	cout << "Mappa vertici:  " << vertexVector.size() << endl;
	
#if 0
	for (int j = 0; j < vertexVector.size(); j++) {
		VertexSE2* tmp =  vertexVector[j].second;
		LaserRobotData* ltmp = dynamic_cast<LaserRobotData*>(tmp->userData());
		if (ltmp)
		{
			cout << "Vertex: " << tmp->id() << " Datal: " << ltmp->paramIndex() << endl;
		}
	}
#endif
// 	std::vector<int> edgesIds(graph->edges().size());
// 	for (int j = 0; j <= edgesIds.size(); j++) 
// 	{
// 		OptimizableGraph::Edge* _e = graph->edges();
// 		EdgeSE3* e3 = dynamic_cast<EdgeSE3*>(_e);
//     if (!e3)
//       continue;
// 		
// 		EdgeSE2* e2 = new EdgeSE2();		
// 		VertexSE3* tmp0 = dynamic_cast<VertexSE3*>(e3->vertices()[0]);
// 		VertexSE3* tmp1 = dynamic_cast<VertexSE3*>(e3->vertices()[1]);
// 		Vertex3to2 vertex0 = vertexVector[tmp0->id()];
// 		Vertex3to2 vertex1 = vertexVector[tmp1->id()];
// 		e2->setVertex(0, vertex0.second);
// 		e2->setVertex(1, vertex1.second);
// 		e2->setMeasurementFromState();
// 		Eigen::Matrix<double, 6,6> m;
// 		m.setIdentity();
// 		e2->setInformation(m);
// 		graphSE2->addEdge(e2);
// 		graphSE2->saveEdge(ofG2O, e2);
// 	}
	
	
		for (OptimizableGraph::EdgeSet::iterator it = graph->edges().begin(); it != graph->edges().end(); it++) {
			OptimizableGraph::Edge* _e = *it;
			EdgeSE3* e3 = dynamic_cast<EdgeSE3*>(_e);
			if (!e3)
				continue;
			
			EdgeSE2* e2 = new EdgeSE2();		
			VertexSE3* tmp0 = dynamic_cast<VertexSE3*>(e3->vertices()[0]);
			VertexSE3* tmp1 = dynamic_cast<VertexSE3*>(e3->vertices()[1]);
			Vertex3to2 vertex0 = vertexVector[tmp0->id()];
			Vertex3to2 vertex1 = vertexVector[tmp1->id()];
			e2->setVertex(0, vertex0.second);
			e2->setVertex(1, vertex1.second);
			e2->setMeasurementFromState();
			Eigen::Matrix<double, 6,6> m;
			m.setIdentity();
			e2->setInformation(m);
			graphSE2->addEdge(e2);
			graphSE2->saveEdge(ofG2O, e2);
	}

	return (0);
	
}