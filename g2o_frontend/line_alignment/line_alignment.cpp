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
#include "g2o/types/slam2d_addons/types_slam2d_addons.h"
#include "g2o_frontend/sensor_data/laser_robot_data.h"
#include "g2o_frontend/basemath/bm_se2.h"
#include "g2o_frontend/ransac/alignment_line2d_linear.h"
#include "g2o_frontend/ransac/ransac.h"
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <boost/concept_check.hpp>

using namespace Eigen;
using namespace std;
using namespace g2o;
using namespace g2o_frontend;


struct lineOfVertex
{
    Line2D line;
    VertexSE2* vertex;
    VertexLine2D* vline;
};

struct lineCorrespondence
{
	double error;
	int lid1;
	int lid2;  
};

typedef std::vector<lineOfVertex, Eigen::aligned_allocator<Vector2d> > LinesSet;
typedef std::pair<LinesSet,LinesSet> LinesForMatching;
typedef std::vector<LinesForMatching> LinesForMatchingVector;
typedef std::vector<lineCorrespondence> LineCorrs;
typedef std::vector<LineCorrs> LineCorrsVector;


bool findCorrespondences(LineCorrsVector& _lcorrsVector, LinesForMatchingVector& _linesSets){
	cout << "....start finding correspondences" << endl; 
	LineCorrs currCorrs;
	double th = 2.f;
	cout << "number of pairs to be matched: " << _linesSets.size() << endl;
	for (int i = 0; i<(int)_linesSets.size(); i++)
	{
		cerr << "* Iteration " << i << endl;		
		currCorrs.clear();
		LinesSet s1 = _linesSets[i].first;
		LinesSet s2 = _linesSets[i].second;
		cout << "number of lines in the first set: " << s1.size() << endl;
		cout << "number of lines in the first set: " << s2.size() << endl;
		for(int j = 0; j < (int)s1.size(); j++)
		{	
			lineCorrespondence lc;
			lc.error = 1e9;
			lc.lid1 = -1;
			lc.lid2 = -1;				
			lineCorrespondence lc_second;
			lc_second.error = 1e9;
			lc_second.lid1 = -1;
			lc_second.lid2 = -1;
						
			Line2D l1 = s1[j].line;
			Vector3d l1_coeff(cos(l1(0)), sin(l1(0)), l1(1));
			
			for(int k = 0; k < (int)s2.size(); k++)
			{
				Line2D l2=s2[k].line;
				Vector3d l2_coeff(cos(l2(0)), sin(l2(0)), l2(1));
				
				double err_n  = abs(l1_coeff[0]-l2_coeff[0] + l1_coeff[1]-l2_coeff[1]);
				double err_rho = abs(l1_coeff[2]-l2_coeff[2]);
	
				double err_sum = err_n + err_rho;
				
				//computing the chi2
				Vector3d err_tot = l1_coeff-l2_coeff;
				err_tot.head<2>() *= 10;
				double err_chi2 = err_tot.squaredNorm();
				
// 				cerr << "- err_sum between frame 0 line "<< j << " and frame 1 line " << k << ":\t" <<  err_sum <<endl;
// 				cerr << "- err_chi2 between frame 0 line "<< j << " and frame 1 line " << k << ":\t" << err_chi2 <<endl<<endl;
				
				if(err_sum < lc.error)
				{
					//considering err_chi2, don't need this if
					if(lc.error < th) {
						lc_second.error = lc.error;
						lc_second.lid1 = lc.lid1;
						lc_second.lid2 = lc.lid2;
					}
					lc.error = err_chi2;//err_sum
					lc.lid1 = j;
					lc.lid2 = k;
				} else if(err_sum < th && err_sum < lc_second.error)
				{
					lc_second.error = err_chi2;//err_sum
					lc_second.lid1 = j;
					lc_second.lid2 = k;
				}
			}
			currCorrs.push_back(lc);
			if(lc_second.error != 1e9){
				currCorrs.push_back(lc_second);
			}
		}
		_lcorrsVector.push_back(currCorrs);
	}
	return true;
}


int main(int argc, char**argv){
	
	string filename;	
	string outfilename;
	g2o::CommandArgs arg;
	arg.paramLeftOver("graph-input", filename , "", "graph file which will be processed", true);
	arg.param("o", outfilename, "newGraph.g2o", "output file name");
	arg.parseArgs(argc, argv);
	ofstream ofG2O(outfilename.c_str());
	
	// graph construction
	typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
	typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
	SlamLinearSolver* linearSolver = new SlamLinearSolver();
	linearSolver->setBlockOrdering(false);
	SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
	OptimizationAlgorithmGaussNewton* solverGauss   = new OptimizationAlgorithmGaussNewton(blockSolver);
	SparseOptimizer * graph = new SparseOptimizer();
	SparseOptimizer * graphline = new SparseOptimizer();
	graph->setAlgorithm(solverGauss);
	graphline->setAlgorithm(solverGauss);
	graph->load(filename.c_str());
	
	// sort the vertices based on the id
	std::vector<int> vertexIds(graph->vertices().size());
	int k=0;
	for (OptimizableGraph::VertexIDMap::iterator it=graph->vertices().begin(); it!= graph->vertices().end(); it ++){
		vertexIds[k++] = (it->first);
	}
	std::sort(vertexIds.begin(), vertexIds.end());
	
	OptimizableGraph::Data* d = 0;
	LaserRobotData* data = 0;
	Isometry2d odom0to1 = Isometry2d::Identity();
	VertexSE2* v_current = 0;
	VertexSE2* v_next = 0;
	OptimizableGraph::EdgeSet es_next;
	LinesSet lvector;
	LinesSet lvector_next;
	LinesForMatchingVector lineMatchingContainer;
	
	//for each vertex
	for (size_t i = 0; i<vertexIds.size(); i++)
	{		
		OptimizableGraph::Vertex* _v = graph->vertex(vertexIds[i]);
		VertexSE2* v = dynamic_cast<VertexSE2*>(_v);
		if (!v)
			continue;
		cout << "***********************************" << endl;
		cout << "***********NEW ITERATION***********" << endl;
		cout << "***********************************" << endl;
		cout << "Current Vertex " << i << endl;
		int vcurr_id = v->id();
		int next_id = -1;
		v_current = v;
		
		//creating the new current vertex for the new graph
		VertexSE2* v_new = new VertexSE2();
		v_new->setEstimate(v_current->estimate());
		v_new->setId(vcurr_id);
		d = v_current->userData();
		
		//getting sensor parameter offset
		while(d) {
			data = dynamic_cast<LaserRobotData*>(d);
			d=d->next();
			if(data){
				Parameter* p = graph->parameters().getParameter(data->paramIndex());
				
				//adding sensor parameter
				if (! graphline->parameters().getParameter(data->paramIndex()))
				{
					ParameterSE3Offset* parameter = dynamic_cast<ParameterSE3Offset*> (p);
					if(!parameter)
						continue;
					cout << "laser parameter offset: \n" << parameter->offset().matrix() << endl;
					parameter->setId(1);
					graphline->parameters().addParameter(p);
				}				
			}
		}
		v_new->addUserData(d);
		graphline->addVertex(v_new);

		//for each edges from the current robot poses
		cout << "*********edges from the current robot poses*********" << endl;
		OptimizableGraph::EdgeSet es = v_current->edges();
		cout << "This vertex has " << es.size() << " edge." << endl; 
		for (OptimizableGraph::EdgeSet::iterator itv = es.begin(); itv != es.end(); itv++) {
			
			EdgeSE2* eSE2 = dynamic_cast<EdgeSE2*>(*itv);
			if (!eSE2)
				continue;		
			
			VertexSE2* tmp0 = dynamic_cast<VertexSE2*>(eSE2->vertices()[0]);
			VertexSE2* tmp1 = dynamic_cast<VertexSE2*>(eSE2->vertices()[1]);
			cout << "- Odom edge from vertex  " << tmp0->id() << " to " << tmp1->id() << endl;
			if(tmp0->id() == vcurr_id)
			{
				//saving the odom edge in the new graph
				graphline->addEdge(eSE2);
				
				odom0to1 = eSE2->measurement().toIsometry();
				cout << "Odometry transformation between the current vertex " << vcurr_id << " and the next one " << tmp1->id() << ":\n" << odom0to1.matrix() << endl;
				v_next = dynamic_cast<VertexSE2*>(eSE2->vertices()[1]);
				next_id = v_next->id();
			}	else {
				v_next = 0;
				next_id = -1;
				odom0to1 = Eigen::Isometry2d::Identity();
				cout << "###Skipping this edge (forward evaluation of the odometry)###" << endl;
			}
		}
		
		if(v_next) {
			es_next = v_next->edges();
			cout << "The next vertex has " << es_next.size() << " edge." << endl; 
		} else {
			es_next.clear();
			cout << "!!!! There is no next vertex --> END OF GRAPH - size of edgeset of v_next:" << es_next.size() << endl;
		}
		
		//line extracted for the current vertex
		int currvertexLine = 0;
		cout << endl << "***Line Extracted from CURRENT frame***" << endl;
		for (OptimizableGraph::EdgeSet::iterator itv = es.begin(); itv != es.end(); itv++) {
			
			EdgeSE2Line2D* el = dynamic_cast<EdgeSE2Line2D*>(*itv);
			if (!el)
				continue;
			
			VertexSE2* tmp = dynamic_cast<VertexSE2*>(el->vertices()[0]);			
			VertexLine2D* vl = dynamic_cast<VertexLine2D*>(el->vertices()[1]);
			
			if (tmp->id() == vcurr_id)
			{
				//creating the new vertexline
				VertexLine2D* vl_new = new VertexLine2D();
				vl_new->setId(vl->id());
				vl_new->setEstimate(vl->estimate());
				vl_new->p1Id = vl->p1Id;
				vl_new->p2Id = vl->p2Id;
				graphline->addVertex(vl_new);			
				//saving the SE2-line edge in the new graph
				graphline->addEdge(el);	
				if(v_next){
					Line2D li = vl->estimate();
					//cout << "- Line " << currvertexLine << ": id "  << vl->id() << ": theta " << li(0) << ", rho " << li(1) /*<< ", estimate: " << vl->estimate().transpose()*/ << endl;
					SE2 odomTransform(odom0to1);
					Line2D tli = odomTransform*li;
					lineOfVertex tliv;
					tliv.line = tli;
					tliv.vertex = v_current;
					tliv.vline = vl;
					//cout << ">> transformed in the next frame: theta " << tliv.line(0) << ", rho " << tliv.line(1) << endl;
					lvector.push_back(tliv);
				}
				currvertexLine++;
			}
			cout << "vector size of lines in the current vertex: " << lvector.size() << endl;

			///new graph stuff
			//creating vertexpoints and saving edges between line and points
			OptimizableGraph::EdgeSet vl_edges = vl->edges();
// 			cout << "This line vertex has " << vl_edges.size() << " edge." << endl; 
			for (OptimizableGraph::EdgeSet::iterator itvl = vl_edges.begin(); itvl != vl_edges.end(); itvl++) {
				EdgeLine2DPointXY* elp = dynamic_cast<EdgeLine2DPointXY*>(*itvl);
				if (!elp){
					continue;}
				
				//creating the new vertexpointxy and the new edge se2-point
				VertexPointXY* vpoint =  dynamic_cast<VertexPointXY*>(elp->vertices()[1]);
				if (vpoint){
					OptimizableGraph::EdgeSet vp_edges = vpoint->edges();
					VertexPointXY* vp_new = new VertexPointXY();
					vp_new->setId(vpoint->id());
					vp_new->setEstimate(vpoint->estimate());
					graphline->addVertex(vp_new);
					
					for (OptimizableGraph::EdgeSet::iterator itvp = vp_edges.begin(); itvp != vp_edges.end(); itvp++) {
						EdgeSE2PointXY* eSEp = dynamic_cast<EdgeSE2PointXY*>(*itvp);
						graphline->addEdge(eSEp);
					}
				}
				//saving the line-point edge in the new graph
				graphline->addEdge(elp);
			}
		}
		
		//saving the lines of the next vertex, to create the correspondences for ransac
		if(v_next) {
			int nextvertexLine = 0;
			cout << endl <<  "***Line Extracted from NEXT frame***" << endl;
			for (OptimizableGraph::EdgeSet::iterator itv_next = es_next.begin(); itv_next != es_next.end(); itv_next++) 
			{
				EdgeSE2Line2D* el_next = dynamic_cast<EdgeSE2Line2D*>(*itv_next);
				if (!el_next)
					continue;
			
				VertexSE2* tmp0_next = dynamic_cast<VertexSE2*>(el_next->vertices()[0]);			
				VertexLine2D* vl_next = dynamic_cast<VertexLine2D*>(el_next->vertices()[1]);
				
				if (tmp0_next->id() == next_id)
				{
					Line2D li_next = vl_next->estimate();
					lineOfVertex liv_next;
					liv_next.line = li_next;
					liv_next.vertex = v_new;
					liv_next.vline = vl_next;
					//cout << "- Line of the next vertex " << nextvertexLine << ": id "  << vl_next->id() << ": theta " << liv_next.line(0) << ", rho " << liv_next.line(1) << endl;
					lvector_next.push_back(liv_next);
					
					nextvertexLine++;
				}
			}
			cout << "vector size of lines in the next vertex: " << lvector_next.size() << endl;
			
			lineMatchingContainer.push_back(make_pair(lvector, lvector_next));
			cout << endl << " ### iteration "  << i << ", SIZE of the container(pair of lines sets): " << lineMatchingContainer.size() << endl << endl;
			
// 			for(size_t uno = 0; uno < (lineMatchingContainer[i].first).size(); uno++){
// 				cout << "primo set:size " << (lineMatchingContainer[i].first).size() << "\n" << (lineMatchingContainer[i].first)[uno] << endl << endl;			}
// 			for(size_t due = 0; due < (lineMatchingContainer[i].second).size(); due++){
// 				cout << "secondo set:size " << (lineMatchingContainer[i].second).size() << "\n" << (lineMatchingContainer[i].second)[due] << endl << endl;			}
		}
		cout << "*********END OF ITERATION*********" << endl << endl << endl;
		
		lvector.clear();
		lvector_next.clear();
	}
	cout << endl << " ### Done with the graph reading, ready for the correspondences finder.. " << endl;
	cout << " ### SIZE OF THE FINAL CONTAINER: " << lineMatchingContainer.size() << endl;
	LineCorrsVector lcorrsVector;
// 	lcorrsVector.resize(lineMatchingContainer.size());	
	//calling find correspondances
	bool resultCorrespondances = findCorrespondences(lcorrsVector, lineMatchingContainer);
	if(resultCorrespondances){
		//call aligner
		cout << "!!! End of findCorrespondances(pair of vertex to be matched), size is " << lcorrsVector.size() << endl;
		cout  << endl;
		for (int c = 0; c < lcorrsVector.size(); c++){
			cout << endl << "*******************************STARTING ALIGNMENT ALGORITHM: ITERATION " << c << "*******************************" << endl << endl;
			LineCorrs currCorrs = lcorrsVector[c];
			CorrespondenceVector correspondences;
			IndexVector indices(currCorrs.size(), 0);
			
			LinesSet s1 = lineMatchingContainer[c].first;
			LinesSet s2 = lineMatchingContainer[c].second;
			
			//computing the ground thruth: the odometry transformation from vertexSE2 of lineSet s1 to vertexSE2 of lineSet s2
			Isometry2d gt = Isometry2d::Identity();
			VertexSE2* v = dynamic_cast<VertexSE2*>(s1[0].vertex);
			for (OptimizableGraph::EdgeSet::iterator itv = v->edges().begin(); itv != v->edges().end(); itv++) {
			
				EdgeSE2* eSE2 = dynamic_cast<EdgeSE2*>(*itv);
				if (!eSE2)
					continue;
				gt = eSE2->measurement().toIsometry();
			}
			cout << "!!! Size of correspondances founded for the " << c << "-th pair of vertex: " << currCorrs.size() << endl;
			
			for (int ci = 0; ci < currCorrs.size(); ci++)
			{
				int vlid1 = -1, vlid2 = -1;
					cerr << "Correspondances position in lines sets: "  <<currCorrs[ci].lid1 << ", " << currCorrs[ci].lid2 << ", with error:  " << currCorrs[ci].error << endl;
					cerr << "ID vertex 1: [" << s1[currCorrs[ci].lid1].vline->id() << "] - ID vertex 2: [" << s2[currCorrs[ci].lid2].vline->id() << "]" << endl;
					vlid1 = graph->vertex(s1[currCorrs[ci].lid1].vline->id())->id();
					vlid2 = graph->vertex(s2[currCorrs[ci].lid2].vline->id())->id();
					cerr << "ID vertex 1: [" << vlid1 << "] - ID vertex 2: [" << vlid2 << "]" << endl;
					EdgeLine2D* eline = new EdgeLine2D;
					VertexLine2D* vli = dynamic_cast<VertexLine2D*>(graph->vertex(s1[currCorrs[ci].lid1].vline->id()));
					VertexLine2D* vlj = dynamic_cast<VertexLine2D*>(graph->vertex(s2[currCorrs[ci].lid2].vline->id()));

					eline->setVertex(0,vli);
					eline->setVertex(1,vlj);
					Correspondence c(eline,100);
					correspondences.push_back(c);
					indices[ci]=ci;
					
// 					for(int i = 0; i < s1.size(); i++){
// 						if(s1[i].vline->id() == currCorrs[ci].lid1){
// 							vlid1 = s1[i].vline->id();
// 							cout << "[1] id vertex: " << vlid1 << endl;
// 						}
// 					}
// 					for(int j = 0; j < s2.size(); j++){
// 						if(s1[j].vline->id() == currCorrs[ci].lid1){
// 							vlid2 = s2[j].vline->id();
// 							cout << "[2] id vertex: " << vlid2 << endl;
// 						}
// 					}
			}
			cerr << "size of correspondances vector: " << correspondences.size() << endl;
			AlignmentAlgorithmLine2DLinear aligner;
			SE2 t0(gt);
// 			SE2 t0(Isometry2d::Identity());
			AlignmentAlgorithmLine2DLinear::TransformType transform = t0;
			bool resultAligner = aligner(transform, correspondences, indices);
			if(resultAligner)
			{
				cerr << "***********FOUND!***********" << endl;
			Isometry2d res = transform.toIsometry();
			Isometry2d _t0 = t0.toIsometry();
			cerr << "ground truth vector: " <<endl;
			cerr << t2v_2d(_t0) << endl;
			cerr << "ground truth: " <<endl;
			cerr << _t0.matrix() << endl;
			cerr << "transform found vector: " <<endl;
			cerr << t2v_2d(res) << endl;
			cerr << "transform found: " <<endl;
			cerr << res.matrix() << endl;
			cerr << "transform error vector: " << endl;
			cerr << t2v_2d(_t0*res) << endl;
			cerr << "transform error: " << endl;
			cerr << (_t0*res).matrix() << endl;
				
			}
			cout << endl << "********************************END OF ALIGNMENT ALGORITHM: ITERATION " << c << "********************************" << endl << endl;
		}
	}
	
	cout << endl;
	//cout << "vectors of lines at the end: current" << lvector.size() << " next " << lvector_next.size() << endl;
	cout << "...saving graph in " << outfilename.c_str() << endl;
	graphline->save(ofG2O);
	ofG2O.close();
	return (0);
}