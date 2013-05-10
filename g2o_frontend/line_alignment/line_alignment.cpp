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
#include <boost/iterator/iterator_concepts.hpp>

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

void mergePointVertex(OptimizableGraph* graph, OptimizableGraph* graphline, VertexPointXY* pNew, VertexPointXY* pOld){
	if (!pNew || !pOld || pNew == pOld)
			return;
	
	int idOld=pOld->id();
	int idNew=pNew->id();
	OptimizableGraph::EdgeSet epoint1 = pOld->edges();
	for (OptimizableGraph::EdgeSet::iterator it_vp1 = epoint1.begin(); it_vp1 != epoint1.end(); it_vp1++)
	{
		EdgeLine2DPointXY* elp1 = dynamic_cast<EdgeLine2DPointXY*>(*it_vp1);
		if(elp1){
			VertexLine2D* line = dynamic_cast<VertexLine2D*>(elp1->vertex(0));
			if (line) {
				if (line->p1Id==idOld)
					line->p1Id = idNew;
				if (line->p2Id==idOld)
					line->p2Id = idNew;
			}
		}
	}
	graphline->mergeVertices(pNew, pOld, true);
}

void mergeLineVertex(OptimizableGraph* graph, OptimizableGraph* graphline, VertexLine2D* lNew, VertexLine2D* lOld){
	if (!lNew || !lOld && lNew == lOld)
			return;
	if (lOld->p1Id > -1 && lNew->p1Id > -1){ // merge the first vertices of both lines
		mergePointVertex(graph, graphline,
						 dynamic_cast<VertexPointXY*>(graph->vertex(lNew->p1Id)),
						 dynamic_cast<VertexPointXY*>(graph->vertex(lOld->p1Id)));
	} else if (lNew->p1Id < 0 && lOld->p1Id > -1){ // just update the id of the keeping line
		cerr << "fuck!!!!!!!!!" << endl;
		lNew->p1Id = lOld->p1Id;
	} 
	graphline->mergeVertices(lNew, lOld, true);
}


bool findCorrespondences(LineCorrsVector& _lcorrsVector, LinesForMatchingVector& _linesSets){
	cout << "....start finding correspondences" << endl; 
	LineCorrs currCorrs;
	double th = 1e3;
	cout << "number of pairs to be matched: " << _linesSets.size() << endl;
	for (int i = 0; i<(int)_linesSets.size(); i++)
	{
		cerr << "* Iteration " << i << endl;		
		currCorrs.clear();
		LinesSet s1 = _linesSets[i].first;
		LinesSet s2 = _linesSets[i].second;
		cout << "number of lines in the first set: " << s1.size() << endl;
		cout << "number of lines in the second set: " << s2.size() << endl;
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
				err_tot.head<2>() *= 1000;
				double err_chi2 = err_tot.squaredNorm();
				
// 				cerr << "- err_sum between frame 0 line "<< j << " and frame 1 line " << k << ":\t" <<  err_sum <<endl;
// 				cerr << "- err_chi2 between frame 0 line "<< j << " and frame 1 line " << k << ":\t" << err_chi2 <<endl<<endl;
				
				if(err_chi2 < lc.error)
				{
					//considering err_chi2, don't need this if
// 					if(lc.error < th) {
// 						lc_second.error = lc.error;
// 						lc_second.lid1 = lc.lid1;
// 						lc_second.lid2 = lc.lid2;
// 					}
					lc.error = err_chi2;
					lc.lid1 = j;
					lc.lid2 = k;
				}/* else if(err_chi2 < th && err_chi2 < lc_second.error)
				{
					lc_second.error = err_chi2;
					lc_second.lid1 = j;
					lc_second.lid2 = k;
				}*/
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

bool updateVertexPointID(SparseOptimizer* graph,SparseOptimizer* graphline, VertexLine2D* vli, VertexLine2D* vlj) {
	//checking if the first extreme point of a line is a common vertex
	bool updated = false;
	VertexPointXY* vlip1 = dynamic_cast<VertexPointXY*>(graph->vertex(vli->p1Id));	
	VertexPointXY* vljp1 = dynamic_cast<VertexPointXY*>(graph->vertex(vlj->p1Id));
	OptimizableGraph::EdgeSet epoint1 = vlip1->edges();
	int count1 = 0;
	for (OptimizableGraph::EdgeSet::iterator it_vp1 = epoint1.begin(); it_vp1 != epoint1.end(); it_vp1++)
	{
		EdgeLine2DPointXY* elp1 = dynamic_cast<EdgeLine2DPointXY*>(*it_vp1);
		if(elp1) count1++;
	}
	
	cerr << "number of edges between point and lines: " << count1<< endl; 
	if(count1 > 1) {
		cerr << "This is a common vertex!!!" << endl;
		{
// 		OptimizableGraph::EdgeSet esvli = vlip1->edges();
// 		for (OptimizableGraph::EdgeSet::iterator it_vli = esvli.begin(); it_vli != esvli.end(); it_vli++)
// 		{
// 			EdgeLine2DPointXY* elpoint = dynamic_cast<EdgeLine2DPointXY*>(*it_vli);
// 			if(!elpoint)
// 				continue;
// 			cout << "removing edge point-line2d" << endl;
// 			graphline->removeEdge(elpoint);
// 			
// 			EdgeSE2PointXY* eSEpoint = dynamic_cast<EdgeSE2PointXY*>(*it_vli);
// 			if(!eSEpoint)
// 				continue;
// 			cout << "removing edge point-se2" << endl;
// 			graphline->removeEdge(eSEpoint);
// 		}
		}
		cout << "removing vertex..." << endl;

		graphline->removeVertex(vlip1);
		vli->p1Id = vlj->p1Id;
		updated = true;
	}
	else updated = false;
	
// 	//checking if the second extreme point of a line is a common vertex
	VertexPointXY* vlip2 = dynamic_cast<VertexPointXY*>(graph->vertex(vli->p2Id));
    VertexPointXY* vljp2 = dynamic_cast<VertexPointXY*>(graph->vertex(vlj->p2Id));
	if(!vlip2)
		updated = false;
	OptimizableGraph::EdgeSet epoint2 = vlip2->edges();
	int count2 = 0;
	for (OptimizableGraph::EdgeSet::iterator it_vp2 = epoint2.begin(); it_vp2 != epoint2.end(); it_vp2++)
	{
		EdgeLine2DPointXY* elp2= dynamic_cast<EdgeLine2DPointXY*>(*it_vp2);
		if(elp2) count2++;
	}
	
	cerr << "number of edges between point 2 and lines: " << count2 << endl; 
	if(count2 > 1) {
		cerr << "The second is a common vertex!!!" << endl;
		if(vli->p2Id != vlj->p2Id){
			cerr << "ceccheccazzzu" << endl;
			vli->p2Id = vlj->p2Id;
// 			cout << "removing second vertex..." << endl;
// 			graphline->removeVertex(vlip2);
			updated = true;
		} 
	}
	else updated = false;
}

#if 0
		ofstream os1("Line1.dat");
		ofstream os2("Line2.dat");
		ofstream os2R("Line2Remapped.dat");

#endif


int main(int argc, char**argv){
	
	string filename;	
	string outfilename;
	g2o::CommandArgs arg;
	arg.param("o", outfilename, "newGraph.g2o", "output file name");
	arg.paramLeftOver("graph-input", filename , "", "graph file which will be processed", true);
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
	
// 	int counttot = 0;
	//for each vertex
	for (size_t i = 0; i<vertexIds.size(); i++)
	{	
		cerr << "num vertici " << vertexIds.size() << endl;
		OptimizableGraph::Vertex* _v = graph->vertex(vertexIds[i]);
		VertexSE2* v = dynamic_cast<VertexSE2*>(_v);
		if (!v)
			continue;
		cout << "***********************************" << endl;
		cout << "***********NEW ITERATION***********" << endl;
		cout << "***********************************" << endl;
// 		cout << "Current Vertex " << i << endl;
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
		cout << "###### edges from the current robot poses ######" << endl;
		OptimizableGraph::EdgeSet es = v_current->edges();
		cout << "This vertex has " << es.size() << " edge." << endl;
// 		int countv = 0;
		for (OptimizableGraph::EdgeSet::iterator itv = es.begin(); itv != es.end(); itv++) {
// 			countv++;
// 			counttot++;
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
// 		cerr << "edgeset del v " << countv << endl;
		
		if(v_next) {
			es_next = v_next->edges();
			cout << "The next vertex has " << es_next.size() << " edge." << endl; 
		} else {
			es_next.clear();
			cout << "!!!! There is no next vertex --> END OF GRAPH - size of edgeset of v_next:" << es_next.size() << endl;
		}
		
		//line extracted for the current vertex
		int currvertexLine = 0;
		cout << endl << "***Extracting lines from CURRENT frame***" << endl;
		for (OptimizableGraph::EdgeSet::iterator itv = es.begin(); itv != es.end(); itv++) {
			
			EdgeSE2Line2D* el = dynamic_cast<EdgeSE2Line2D*>(*itv);
			if (!el)
				continue;
			
			VertexSE2* tmp = dynamic_cast<VertexSE2*>(el->vertices()[0]);			
			VertexLine2D* vl = dynamic_cast<VertexLine2D*>(el->vertices()[1]);
			
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
					if (!graphline->vertex(vp_new->id())) {
						bool fuck2 = graphline->addVertex(vp_new);
						if(!fuck2) cout << "perchè !?!?!?!?" << endl;
					}
					
					for (OptimizableGraph::EdgeSet::iterator itvp = vp_edges.begin(); itvp != vp_edges.end(); itvp++) {
						EdgeSE2PointXY* eSEp = dynamic_cast<EdgeSE2PointXY*>(*itvp);
						graphline->addEdge(eSEp);
					}
				}
				//saving the line-point edge in the new graph
				graphline->addEdge(elp);
			}
			
			//saving the lines adding vertexline t the new graph
			if (tmp->id() == vcurr_id)
			{
				//creating the new vertexline
				VertexLine2D* vl_new = new VertexLine2D();
				vl_new->setId(vl->id());
				vl_new->setEstimate(vl->estimate());
				vl_new->p1Id = vl->p1Id;
				vl_new->p2Id = vl->p2Id;
				bool fuck = graphline->addVertex(vl_new);
				if(!fuck) cout << "perchè line !?!?!?!?" << endl;

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
			

		}
		cout << "Saving lines of the current vertex, size is: " << lvector.size() << endl;
		
		//saving the lines of the next vertex, to create the correspondences for ransac
		if(v_next) {
			int nextvertexLine = 0;
			cout << endl <<  "***Extracting lines from NEXT frame***" << endl;
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
			cout << "Saving lines of the next vertex, size is:" << lvector_next.size() << endl;
			
			lineMatchingContainer.push_back(make_pair(lvector, lvector_next));
			cout << endl << " ### iteration "  << i << ", SIZE of the container(pair of lines sets): " << lineMatchingContainer.size() << endl << endl;
// 				debug
// 			for(size_t uno = 0; uno < (lineMatchingContainer[i].first).size(); uno++){
// 				cout << "primo set:size " << (lineMatchingContainer[i].first).size() << "\n" << (lineMatchingContainer[i].first)[uno] << endl << endl;			}
// 			for(size_t due = 0; due < (lineMatchingContainer[i].second).size(); due++){
// 				cout << "secondo set:size " << (lineMatchingContainer[i].second).size() << "\n" << (lineMatchingContainer[i].second)[due] << endl << endl;			}
		}
		cout << "*********END OF ITERATION*********" << endl << endl << endl;
		
		lvector.clear();
		lvector_next.clear();
	}
// 	cerr << "edgeset tot " << counttot << endl;
	
	cout << endl << " ### Done with the graph reading, ready for the correspondences finder.. " << endl;
	cout << " ### SIZE OF THE FINAL CONTAINER: " << lineMatchingContainer.size() << endl;
	LineCorrsVector lcorrsVector;
	lcorrsVector.reserve(lineMatchingContainer.size());	
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
				cerr << "Correspondances position in lines sets: "  <<currCorrs[ci].lid1 << ", " << currCorrs[ci].lid2 << ", with error:  " << currCorrs[ci].error << endl;
// 				debug
// 				int vlid1 = -1, vlid2 = -1;
// 				cerr << "ID vertex 1: [" << s1[currCorrs[ci].lid1].vline->id() << "] - ID vertex 2: [" << s2[currCorrs[ci].lid2].vline->id() << "]" << endl;
// 				vlid1 = graph->vertex(s1[currCorrs[ci].lid1].vline->id())->id();
// 				vlid2 = graph->vertex(s2[currCorrs[ci].lid2].vline->id())->id();
// 				cerr << "ID vertex 1: [" << vlid1 << "] - ID vertex 2: [" << vlid2 << "]" << endl;
				EdgeLine2D* eline = new EdgeLine2D;
				VertexLine2D* vli = dynamic_cast<VertexLine2D*>(graph->vertex(s1[currCorrs[ci].lid1].vline->id()));
				VertexLine2D* vlj = dynamic_cast<VertexLine2D*>(graph->vertex(s2[currCorrs[ci].lid2].vline->id()));
				
				eline->setVertex(0,vli);
				eline->setVertex(1,vlj);
				Matrix2d info;
				info << 100, 0, 0, 1;
				eline->setInformation(info);
				Correspondence c(eline,100);
				correspondences.push_back(c);
				indices[ci]=ci;
				
				//debug
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
			
			
// 			SE2 t0(Isometry2d::Identity());
// 			AlignmentAlgorithmLine2DLinear aligner;
// 			AlignmentAlgorithmLine2DLinear::TransformType transform = t0;
// 			bool resultAligner = aligner(transform, correspondences, indices);
			SE2 t0(gt);
			RansacLine2DLinear ransac;
			CorrespondenceValidatorPtrVector validators;
			ransac.correspondenceValidators()=validators;
			ransac.setCorrespondences(correspondences);
			ransac.setMaxIterations(1000);
			ransac.setInlierErrorThreshold(1.);
			ransac.setInlierStopFraction(0.5);
			RansacLine2DLinear::TransformType transform = t0;
			ScopeTime t("ransac aligned");
			std::vector<int> myInliers;
			bool resultRansac = ransac(transform, myInliers);
			
			
			if(resultRansac)
			{
				cerr << "***********FOUND!***********" << endl;
				Isometry2d res = transform.toIsometry();
				Isometry2d _t0 = t0.toIsometry();
				cerr << endl;
	// 			cerr << "ground truth vector: " <<endl;
	// 			cerr << t2v_2d(_t0) << endl;
// 				cerr << "ground truth: " <<endl;
// 				cerr << _t0.matrix() << endl;
// 				cerr << endl;
				//cerr << "transform found vector: " <<endl;
				//cerr << t2v_2d(res) << endl;
	// 			cerr << "transform found: " <<endl;
	// 			cerr << res.matrix() << endl;
				cerr << endl;
				cerr << "transform error : " << endl;
				cerr << t2v_2d(_t0*res) << endl;
	// 			cerr << "transform error: " << endl;
	// 			cerr << (_t0*res).matrix() << endl;
				
				vector<double> err = ransac.errors();
				cout << "Erros size: " << err.size() << endl;
				for (int h = 0; h < err.size(); h++){
					double erri = err[h];
					cout << "error of " << h << "-th correspondance: " << erri << endl;
				}
				
				//book keeping (updating the vertex point id of the correspondences for common vertex)
// 				for (int ci = 0; ci < currCorrs.size(); ci++)
// 				{
// 					VertexLine2D* vli = dynamic_cast<VertexLine2D*>(graph->vertex(s1[currCorrs[ci].lid1].vline->id()));
// 					VertexLine2D* vlj = dynamic_cast<VertexLine2D*>(graph->vertex(s2[currCorrs[ci].lid2].vline->id()));
//                     updateVertexPointID(graph, graphline, vli, vlj);
// 				}

				//merging vertexes and lines
				for (int ci = 0; ci < currCorrs.size(); ci++)
				{
					VertexLine2D* vli = dynamic_cast<VertexLine2D*>(graph->vertex(s1[currCorrs[ci].lid1].vline->id()));
					VertexLine2D* vlj = dynamic_cast<VertexLine2D*>(graph->vertex(s2[currCorrs[ci].lid2].vline->id()));
// 					cout << "prendo linee" << endl;
#if 0
						//plotting lines frame i and lines frame j remapped with the transform found
						Vector2d line1 = Vector2d(vli->estimate());
						VertexPointXY* vpl1_1 = dynamic_cast<VertexPointXY*>(graphline->vertex(s1[currCorrs[ci].lid1].vline->p1Id));
						VertexPointXY* vpl2_1 = dynamic_cast<VertexPointXY*>(graphline->vertex(s1[currCorrs[ci].lid1].vline->p2Id));
						Vector2d nline1(cos(line1(0)), sin(line1(0)));
						Vector2d pmiddle1 = nline1*line1(1);
						Vector2d t1(-nline1.y(), nline1.x());
						double l1_1,l2_1 = 10;
						l1_1 = t1.dot(vpl1_1->estimate()-pmiddle1);
						l2_1 = t1.dot(vpl2_1->estimate()-pmiddle1);
						Vector2d p1line1 = pmiddle1 + t1*l1_1;
						Vector2d p2line1 = pmiddle1 + t1*l2_1;
						os1 << p1line1.transpose() << endl;
						os1 << p2line1.transpose() << endl;
						os1 << endl;
						os1 << endl;
						os1.flush();
						
						Vector2d line2Remapped = Vector2d(transform*(vlj->estimate()));
						VertexPointXY* vpl1_2 = dynamic_cast<VertexPointXY*>(graph->vertex(s2[currCorrs[ci].lid2].vline->p1Id));
						VertexPointXY* vpl2_2 = dynamic_cast<VertexPointXY*>(graph->vertex(s2[currCorrs[ci].lid2].vline->p2Id));
						Vector2d vpl1_2R = transform*(vpl1_2->estimate());
						Vector2d vpl2_2R = transform*(vpl2_2->estimate());
						Vector2d nline2R(cos(line2Remapped(0)), sin(line2Remapped(0)));
						Vector2d pmiddle2 = nline2R*line2Remapped(1);
						Vector2d t2R(-nline2R.y(), nline2R.x());
						double l1_2R,l2_2R = 10;
						l1_2R = t2R.dot(vpl1_2R - pmiddle2);
						l2_2R = t2R.dot(vpl2_2R - pmiddle2);
						Vector2d p1line2R = pmiddle2 + t2R*l1_2R;
						Vector2d p2line2R = pmiddle2 + t2R*l2_2R;
						os2R << p1line2R.transpose() << endl;
						os2R << p2line2R.transpose() << endl;
						os2R << endl;
						os2R << endl;
						os2R.flush();
						
						Vector2d line2 = Vector2d(vlj->estimate());
						VertexPointXY* vpl1_2_ = dynamic_cast<VertexPointXY*>(graph->vertex(s2[currCorrs[ci].lid2].vline->p1Id));
						VertexPointXY* vpl2_2_ = dynamic_cast<VertexPointXY*>(graph->vertex(s2[currCorrs[ci].lid2].vline->p2Id));
						Vector2d nline2(cos(line2(0)), sin(line2(0)));
						Vector2d pmiddle2_ = nline2*line2(1);
						Vector2d t2(-nline2.y(), nline2.x());
						double l1_2,l2_2 = 10;
						l1_2 = t2.dot(vpl1_2_->estimate() - pmiddle2_);
						l2_2 = t2.dot(vpl2_2_->estimate() - pmiddle2_);
						Vector2d p1line2 = pmiddle2_ + t2*l1_2;
						Vector2d p2line2 = pmiddle2_ + t2*l2_2;
						os2 << p1line2.transpose() << endl;
						os2 << p2line2.transpose() << endl;
						os2 << endl;
						os2 << endl;
						os2.flush();
#endif
					mergeLineVertex(graph, graphline, vlj, vli);
					
// 					VertexPointXY* vlip1 = dynamic_cast<VertexPointXY*>(graph->vertex(vli->p1Id));	
// 					VertexPointXY* vljp1 = dynamic_cast<VertexPointXY*>(graph->vertex(vlj->p1Id));
// 					
// 					if(vlip1->id() != vljp1->id()) {
// 						bool resultmergePoint1 = graphline->mergeVertices(vljp1, vlip1 ,true);
// 						cout << "merging point1: " << resultmergePoint1 << endl;
// 					}
// 						
// 					VertexPointXY* vlip2 = dynamic_cast<VertexPointXY*>(graph->vertex(vli->p2Id));
// 					VertexPointXY* vljp2 = dynamic_cast<VertexPointXY*>(graph->vertex(vlj->p2Id));
// 					if(!vlip2 && !vljp2) cerr << "che cribbio succede???" << endl;
// 					if(vlip2 || vljp2) {
// 						if(vlip2 && !vljp2){
// 							cerr << "imposible ploplio --> agiolnelei indice" << endl;
// 							vlj->p2Id = vlip2->id();
// 						} else if(!vlip2 && vljp2) {
// 							cerr << "nun faccio una mazza" << endl;							
// 						} else if(vlip2 && vljp2 && vlip2->id() != vljp2->id()){
// 							bool resultmergePoint2 = graphline->mergeVertices(vljp2, vlip2 ,true);
// 							cout << "merging point2: " << resultmergePoint2 << endl;
// 						}
// 					}
// 					bool resultmergeLine = graphline->mergeVertices(vlj, vli ,true);
// 					cout << "merging line: " << resultmergeLine << endl;
						
						

					}
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

