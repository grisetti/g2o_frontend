/*
 * toGraphSE2.cpp
 *
 *  Created on: Jan 22, 2013
 *      Author: Martina
 */
#include <fstream>
#include <iomanip>

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
#include "g2o_frontend/sensor_data/laser_robot_data.h"
#include "g2o_frontend/sensor_data/rgbd_data.h"
#include "g2o_frontend/basemath/bm_se2.h"


using namespace std;
using namespace g2o;

typedef std::pair<g2o::VertexSE3*, g2o::VertexSE2*> Vertex3to2;
typedef std::vector<Vertex3to2> v3Mapv2;

int main(int argc, char**argv){
	
  string filename;	
  string outfilename;
	g2o::CommandArgs arg;
	arg.paramLeftOver("graph-input", filename , "", "graph file which will be processed", true);
	arg.param("o", outfilename, "graphSE2.g2o", "output file name");
  arg.parseArgs(argc, argv);
	ofstream ofG2O(outfilename.c_str());
	
  // graph construction
//   typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
//   typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
//   SlamLinearSolver* linearSolver = new SlamLinearSolver();
//   linearSolver->setBlockOrdering(false);
//   SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
//   OptimizationAlgorithmGaussNewton* solverGauss   = new OptimizationAlgorithmGaussNewton(blockSolver);
  OptimizableGraph * graph = new OptimizableGraph(); //SparseOptimizer
  OptimizableGraph * graphSE2 = new OptimizableGraph();
//   graph->setAlgorithm(solverGauss);
//   graphSE2->setAlgorithm(solverGauss);
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
// 	Sensor* sensor = 0;
// 	OptimizableGraph* graphSE2 = new OptimizableGraph();
	for (size_t i = 0; i<vertexIds.size(); i++)
	{
    OptimizableGraph::Vertex* _v = graph->vertex(vertexIds[i]);
    VertexSE3* v3 = dynamic_cast<VertexSE3*>(_v);
    if (!v3)
      continue;		
		
		VertexSE2* v2 = new VertexSE2();
		v2->setEstimate(iso3toSE_2d(v3->estimate()));
		v2->setId(v3->id());
		d = v3->userData();		
	  
		while(d) {	
			data = dynamic_cast<LaserRobotData*>(d);
			d=d->next();
			if(data)
			{
// 				cout << "i: " << i << endl;
				v2->addUserData(data);
				
				//adding sensor parameter
				Parameter* p = graph->parameters().getParameter(data->paramIndex());
				if (! graphSE2->parameters().getParameter(data->paramIndex()))
				{// TODO changing in SE2
					ParameterSE3Offset* parameter = dynamic_cast<ParameterSE3Offset*> (p);
					parameter->setId(1);
					graphSE2->parameters().addParameter(p);
					graphSE2->saveParameter(ofG2O, p);
				}
				//adding sensor parameter, using SensorData, not working
// 				sensor = data->getSensor();
// 				assert (!sensor && "!");
// 				cout << "pippo3" << endl;		
// 				Parameter* parameter = sensor->getParameter();
// 				assert (!parameter && "!");
// 				
// 				if (! graphSE2->parameters().getParameter(parameter->id())){
// 					cout << "pippo4" << endl; 
// 					graphSE2->parameters().addParameter(parameter);
// 					cout << "pippo5" << endl;
// 					graphSE2->saveParameter(ofG2O, parameter);
// 				}
				
				graphSE2->addVertex(v2);
// 				graphSE2->saveVertex(ofG2O, v2);		
			}
			
		}
		vertexVector.push_back(make_pair(v3, v2));	
	}
		
// 	cout << "Map vertices:  " << vertexVector.size() << endl;
cout << "Graph vertices: " << graphSE2->vertices().size() << endl;
cout << "GraphSE2 vertices: " << graphSE2->vertices().size() << endl;
	
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
	
	//adding edges: to do meanwhile I save the vertices
		for (OptimizableGraph::EdgeSet::iterator it = graph->edges().begin(); it != graph->edges().end(); it++) {
			
			EdgeSE3* e3 = dynamic_cast<EdgeSE3*>(*it);
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
			Eigen::Matrix<double, 3,3> info;
			info.setIdentity()*1000;
			e2->setInformation(info);
			graphSE2->addEdge(e2);
// 			graphSE2->saveEdge(ofG2O, e2);
	}

	cout << "Graph edges: " << graph->edges().size() << endl;
	cout << "GraphSE2 edges: " << graphSE2->edges().size() << endl;

	cout << "...saving graph in " << outfilename.c_str() << endl;
	graphSE2->save(ofG2O);
	ofG2O.close();
	return (0);
}
