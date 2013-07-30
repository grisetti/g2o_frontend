#ifndef _ALIGNMENT_UTILS
#define _ALIGNMENT_UTILS


#include <Eigen/Geometry>
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
// #include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/types/slam2d_addons/types_slam2d_addons.h"
#include "g2o_frontend/sensor_data/laser_robot_data.h"
#include "g2o_frontend/basemath/bm_se2.h"
#include "g2o_frontend/ransac/alignment_line2d_linear.h"
#include "g2o_frontend/ransac/ransac.h"


using namespace Eigen;
using namespace std;
using namespace g2o;
using namespace g2o_frontend;


struct lineOfVertex {
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

struct LineLengthComparator{
    inline bool operator()(const VertexLine2D* v1, const VertexLine2D* v2 ){
        const OptimizableGraph* inputGraph=v1->graph();
        Eigen::Vector2d p11=dynamic_cast<const VertexPointXY*>(inputGraph->vertex(v1->p1Id))->estimate();
        Eigen::Vector2d p12=dynamic_cast<const VertexPointXY*>(inputGraph->vertex(v1->p2Id))->estimate();
        Eigen::Vector2d p21=dynamic_cast<const VertexPointXY*>(inputGraph->vertex(v2->p1Id))->estimate();
        Eigen::Vector2d p22=dynamic_cast<const VertexPointXY*>(inputGraph->vertex(v2->p2Id))->estimate();
        double l1=(p11-p12).squaredNorm();
        double l2=(p21-p22).squaredNorm();
        return l1>l2;
    }
};

typedef std::vector<lineOfVertex, Eigen::aligned_allocator<Vector2d> > LinesSet;
typedef std::pair<LinesSet,LinesSet> LinesForMatching;
typedef std::vector<LinesForMatching> LinesForMatchingVector;
typedef std::vector<lineCorrespondence> LineCorrs;
typedef std::vector<LineCorrs> LineCorrsVector;


double computeError(Vector3d& l1_coeff, Vector3d& l2_coeff, int weight) {
    // 	    double err_n  = abs(l1_coeff[0]-l2_coeff[0] + l1_coeff[1]-l2_coeff[1]);
    // 	    double err_rho = abs(l1_coeff[2]-l2_coeff[2]);
    // 	    double err_sum = err_n + err_rho;
    Vector3d err_tot = l1_coeff-l2_coeff;
    err_tot.head<2>() *= weight;
    return err_tot.squaredNorm();
}

bool lengthError(VertexLine2D* vl1, VertexLine2D* vl2){
    bool result = false;
//    double lenghtThreshold = 0.5;
    const OptimizableGraph* inputGraph=vl1->graph();
    Eigen::Vector2d p11=dynamic_cast<const VertexPointXY*>(inputGraph->vertex(vl1->p1Id))->estimate();
    Eigen::Vector2d p12=dynamic_cast<const VertexPointXY*>(inputGraph->vertex(vl1->p2Id))->estimate();
    double d1=(p11-p12).norm();

    Eigen::Vector2d p21=dynamic_cast<const VertexPointXY*>(inputGraph->vertex(vl2->p1Id))->estimate();
    Eigen::Vector2d p22=dynamic_cast<const VertexPointXY*>(inputGraph->vertex(vl2->p2Id))->estimate();
    double d2=(p21-p22).norm();

    //checking if the lines have similar lenght
    if(d1>d2)
        result = (d1/d2) < 1.3;
    else
        result = (d2/d1) < 1.3;

//    result = fabs(d1-d2) > lenghterr;

    if(!result){
        cout << "lenght diff NOT OK: " << fabs(d1-d2) << endl;
    }
    else cout << "lenght diff ok: " << fabs(d1-d2) << endl;

    return result;

}

bool findCorrespondences(LineCorrs& _currCorrs,  LinesForMatching& _pairLinesSet){

    double distanceThreshold = 0.5;
    LinesSet s1 = _pairLinesSet.first;
    LinesSet s2 = _pairLinesSet.second;
    cerr << "number of lines in the first set: " << s1.size() << endl;
    cerr << "number of lines in the second set: " << s2.size() << endl;
    for(size_t j = 0; j < s1.size(); j++)
    {
        lineCorrespondence lc;
        lc.error = 1e9;
        lc.lid1 = -1;
        lc.lid2 = -1;

        VertexLine2D* vl1 = s1[j].vline;
        const OptimizableGraph* inputGraph=vl1->graph();
        Eigen::Vector2d p11=dynamic_cast<const VertexPointXY*>(inputGraph->vertex(vl1->p1Id))->estimate();
        Eigen::Vector2d p12=dynamic_cast<const VertexPointXY*>(inputGraph->vertex(vl1->p2Id))->estimate();

        Line2D l1 = s1[j].line;
        Vector3d l1_coeff(cos(l1(0)), sin(l1(0)), l1(1));

        for(size_t k = 0; k < s2.size(); k++)
        {

            VertexLine2D* vl2 = s2[k].vline;
            Eigen::Vector2d p21=dynamic_cast<const VertexPointXY*>(inputGraph->vertex(vl2->p1Id))->estimate();
            Eigen::Vector2d p22=dynamic_cast<const VertexPointXY*>(inputGraph->vertex(vl2->p2Id))->estimate();

            //checking the lenght of the two lines
            bool similarLenght = lengthError(vl1, vl2);
            if (!similarLenght)
                continue;

            //checking the distance between the 2 lines
            Eigen::Vector2d pm1 = (p11+p12)*0.5;
            Eigen::Vector2d pm2 = (p21+p22)*0.5;
            double dmiddle = (pm1-pm2).norm();
            double dfirst = (p11-p21).norm();
            double dsecond = (p21-p22).norm();
            double daverage = (dmiddle + dfirst + dsecond)/3;
            if(daverage > distanceThreshold) {
                cout << "porca di quella troia!!!!!" << daverage << endl;
                continue;
            }

            Line2D l2 = s2[k].line;
            Vector3d l2_coeff(cos(l2(0)), sin(l2(0)), l2(1));

            //computing the chi2
            int weight = 10;
            double err_chi2 = computeError(l1_coeff, l2_coeff, weight);
            /*
        cerr << "- err_sum between frame 0 line "<< j << " and frame 1 line " << k << ":\t" <<  err_sum <<endl;
        cerr << "- err_chi2 between frame 0 line "<< j << " and frame 1 line " << k << ":\t" << err_chi2 <<endl<<endl;*/

            if(err_chi2 < lc.error)
            {
                lc.error = err_chi2;
                lc.lid1 = j;
                lc.lid2 = k;
//                cout << "SIGH!!!!!!!!" << endl;
            }
        }
        if(lc.error != 1e9)
            _currCorrs.push_back(lc);
    }
    cerr << "Number of correspondances found: " << _currCorrs.size() << endl;
    return true;
}


void mergePointVertex(OptimizableGraph* graph, VertexPointXY* pNew, VertexPointXY* pOld){
	if (!pNew || !pOld || pNew == pOld)
	    return;
	int idNew=pNew->id();
	int idOld=pOld->id();
	
	OptimizableGraph::EdgeSet epoint1 = pOld->edges();
	for (OptimizableGraph::EdgeSet::iterator it_vp1 = epoint1.begin(); it_vp1 != epoint1.end(); it_vp1++)
	{
	    EdgeLine2DPointXY* elp1 = dynamic_cast<EdgeLine2DPointXY*>(*it_vp1);
	    if(elp1){
            VertexLine2D* line = dynamic_cast<VertexLine2D*>(elp1->vertex(0));
            if (line) {
                if (line->p1Id==idOld){
                    cout << "book keeping point1" << endl;
                    line->p1Id = idNew;
                }
                cout << "AAAAAAAAA" << endl;
                // il secondo point vertex viene cancellato solo se è il primo di un vertice linea adiacente
                if (line->p2Id==idOld){
                    cout << "book keeping point2" << endl;
                    line->p2Id = idNew;
                }
            }
	    }
	}
	cout << "merdging punti: " << graph->mergeVertices(pNew, pOld, true) << endl;
}

bool mergeLineVertex(OptimizableGraph* graph, VertexLine2D* lNew, VertexLine2D* lOld){
    bool merdging = false;
	if ((!lNew || !lOld) && (lNew == lOld))
	    return merdging;
	
	VertexPointXY* vlOldp2 = dynamic_cast<VertexPointXY*>(graph->vertex(lOld->p2Id));
	int count = 0;
	if(vlOldp2)
	{
	    OptimizableGraph::EdgeSet epoint2 = vlOldp2->edges();
	    count = 0;
	    for (OptimizableGraph::EdgeSet::iterator it_vp2 = epoint2.begin(); it_vp2 != epoint2.end(); it_vp2++)
	    {
		    EdgeLine2DPointXY* elp2= dynamic_cast<EdgeLine2DPointXY*>(*it_vp2);
		    if(elp2) count++;
	    }
	}
	if(count == 1 && lNew->p2Id > -1){
		cerr << "merging the second point vertices..." << endl;
		mergePointVertex(graph, dynamic_cast<VertexPointXY*>(graph->vertex(lNew->p2Id)), 
						    vlOldp2);
	}	
	    
	if (lOld->p1Id > -1 && lNew->p1Id > -1){ // merge the first vertices of both lines
		cerr << "merging the first point vertices..." << endl;
		mergePointVertex(graph, dynamic_cast<VertexPointXY*>(graph->vertex(lNew->p1Id)),
						    dynamic_cast<VertexPointXY*>(graph->vertex(lOld->p1Id)));
		
	} else if (lOld->p1Id > -1 && lNew->p1Id < 0){ // just update the point id of the keeping line
		cerr << "the new line doesn't have the first point: updating the first point id of the keeping line" << endl;
		lNew->p1Id = lOld->p1Id;
	}

// 	cout << "aaaaa" << graph->vertex(lNew->id()) << endl;
	merdging = graph->mergeVertices(lNew, lOld, true);
	return merdging;
}
bool ransacExec(CorrespondenceValidatorPtrVector& validators,
		CorrespondenceVector& correspondanceVector,
		vector<int>& inliers,
		RansacLine2DLinear::TransformType& transform,
		int iterations,
		float inliersThreshold,
		float inliersStopFraction,
		vector<double>& err,
        bool debug = false,
        int bestFriendFilter = 0)
{
    g2o_frontend::RansacLine2DLinear ransac;
    ransac.correspondenceValidators()=validators;
    ransac.setCorrespondences(correspondanceVector);
    ransac.setMaxIterations(iterations);
    ransac.setInlierErrorThreshold(inliersThreshold);
    ransac.setInlierStopFraction(inliersStopFraction);
    if(ransac(transform, inliers, debug, bestFriendFilter)) {
        err = ransac.errors();
        correspondanceVector = ransac.correspondences();
        cerr << "Ransac done!" << endl;
        return true;
    }
    else
    {
        cerr << "Ransac failed!" << endl;
        return false;
    }
}

void deleteVertices(int first, int last, OptimizableGraph* graph) {
    for (int i = first; i <= last; i++)
    {
//        cout << "i : " << i << endl;
        OptimizableGraph::Vertex* _v = graph->vertex(i);
        if(!_v)
            continue;
        VertexSE2* v = dynamic_cast<VertexSE2*>(_v);
        if(!v)
            continue;
//        cout << "id vertex to be deleted " << v->id() << endl;
        OptimizableGraph::EdgeSet es = v->edges();
        for (OptimizableGraph::EdgeSet::iterator it = es.begin(); it != es.end(); it++)
        {
            EdgeSE2Line2D* el = dynamic_cast<EdgeSE2Line2D*>(*it);
            if (!el)
                continue;

//            VertexSE2* tmp = dynamic_cast<VertexSE2*>(el->vertices()[0]);
            VertexLine2D* vl = dynamic_cast<VertexLine2D*>(el->vertices()[1]);
            if (!vl)
                continue;
            VertexPointXY* vpl1 = dynamic_cast<VertexPointXY*>(graph->vertex(vl->p1Id));
            if (vpl1)
                graph->removeVertex(vpl1);
            VertexPointXY* vpl2 = dynamic_cast<VertexPointXY*>(graph->vertex(vl->p2Id));
            if (vpl2)
                graph->removeVertex(vpl2);

            graph->removeVertex(vl);
        }
//        cout << " Vertex-se2 deleted: "<< graph->removeVertex(v) << endl;
    }
}

void saveOdom0to1(VertexSE2* v_current, VertexSE2* v_next, Isometry2d& odom0to1, EdgeSE2* eSE2) {

    cout << "###### edges from the current robot poses ######" << endl;
    OptimizableGraph::EdgeSet es = v_current->edges();
    cout << "This vertex has " << es.size() << " edge." << endl;
	
    for (OptimizableGraph::EdgeSet::iterator itv = es.begin(); itv != es.end(); itv++) {
	eSE2 = dynamic_cast<EdgeSE2*>(*itv);
	if (!eSE2)
		continue;
	
	VertexSE2* tmp0 = dynamic_cast<VertexSE2*>(eSE2->vertices()[0]);
	VertexSE2* tmp1 = dynamic_cast<VertexSE2*>(eSE2->vertices()[1]);
	cout << "- Odom edge from vertex  " << tmp0->id() << " to " << tmp1->id() << endl;
	if(tmp0->id() == v_current->id())
	{

		odom0to1 = eSE2->measurement().toIsometry();
		cout << "Odometry transformation between the current vertex " << v_current->id() << " and the next one " << tmp1->id() << ":\n" << odom0to1.matrix() << endl;
		v_next = dynamic_cast<VertexSE2*>(tmp1);
		cerr << "ahhhh " << v_next->id() << endl;
		cerr << "ahhhhhhhh " << v_next << endl;
		cerr << "ahhhhhhhh " << eSE2 << endl;
		//next_id = v_next->id();
	} else{
		v_next = 0;
		//next_id = -1;
		odom0to1 = Eigen::Isometry2d::Identity();
		cout << "###Skipping this edge (forward evaluation of the odometry)###" << endl;
	}
    }
}

//INPUT
//  the graph
//  the initial vertex
//OUTPUT
//  the next vertex in the odometry
// void get_next_vertexSE3(OptimizableGraph* graph,VertexSE3* v1, VertexSE3* v2,Isometry3d &odometry,EdgeSE3* eSE3)
// {
//     OptimizableGraph::Vertex* _vTEMP;
// 
//     //accedo ad ogni edge di quel vertice
//     OptimizableGraph::EdgeSet e = v1->edges();
//     //di quei edge accedo ad ogni vertice successivo
//     for (HyperGraph::EdgeSet::iterator it = e.begin(); it!=e.end(); it++)
//     {
// 
//         HyperGraph::Edge* _e = *it;
//         //accedo solo a quelli SE3 per sapere su quale vertice spostarmi
//         eSE3 =dynamic_cast< EdgeSE3*>(_e);
// 
//         if(eSE3)
//         {
//             //accedo al vertice successivo sul quale andare
//             VertexSE3* nextVertex = dynamic_cast<  VertexSE3*>(eSE3->vertex(1));
// 
//             //verifico che il vertice che ho recuperato dall'Edge è effettivamente
//             //- di tipo EdgeSE3 (verificato in precedenza)
//             //- il suo id è differente da quello di partenza
// 
//             if(nextVertex->id()!=v1->id() && nextVertex)
//             {
//                 cout << "mi muovo da - a"<<endl;
//                 cout <<"V[" <<v1->id() << "] -> V[" << nextVertex->id() <<"] "<< endl;
//                 _vTEMP=graph->vertex(nextVertex->id());
// 
//                 //se va tutto bene a questo punto l'odometria deve essere la stessa della trasformata iniziale
//                 odometry=eSE3->measurement();
//                 cout << "Odometria letta dal grafo:"<<endl;
//                 printVector6dAsRow(g2o::internal::toVectorMQT(odometry),1);
//                 //cout << "Trasformata letta da file:"<<endl;
//                 //printVector6dAsRow(g2o::internal::toVectorMQT(trasformata),1);
// 
//                 //outgraph.addEdge(eSE3);
//                 //_v è il nuovo vertice su cui mi devo spostare
//                 v2=dynamic_cast<VertexSE3*>(_vTEMP);
//             }
//         }
//     }
// }
// 
// 
// 
// void compute_Correspondance_Vector(vector<container> &c1,
//                                    vector<container> &c2,
//                                    vector<container> &c2R,
//                                    CorrespondenceVector &correspondanceVector)
// {
//     // C1 plane container 1
//     // C2 plane container 2
//     // C2R plane container 2
// 
//     for(int i=0;i<c1.size();i++)
//     {
// 
//         Plane3D p1=((c1.at(i)).plane)->estimate();
// 
//         for(int j=0;j<c2.size();j++)
//         {
// 
//             Plane3D p2=((c2.at(j)).plane)->estimate();
//             Plane3D p2R=((c2R.at(j)).plane)->estimate();
// 
//             double error = computeError(p1,p2R);
// 
// 
// 
//             //DEBUG INFO ----
//             printPlaneCoeffsAsRow(p1);
//             cout << " <> ";
//             printPlaneCoeffsAsRow(p2);
//             cout <<" ["<<error<<"]"<<endl;
//             //DEBUG INFO ----
// 
//             //FILLING CORRESPONDANCE VECTOR
//             EdgePlane* eplane = new EdgePlane;
// 
//             eplane->setVertex(0,c1.at(i).plane);
//             eplane->setVertex(1,c2.at(i).plane);
//             g2o_frontend::Correspondence corr(eplane,error);
//             if(error<1)
//                 correspondanceVector.push_back(corr);
// 
//         }
//         cout << endl;
//     }
// }
// 
// void merge_vertices(OptimizableGraph* graph,CorrespondenceVector &correspondanceVector)
// {
//     for(int i =0;i<correspondanceVector.size();i++)
//     {
//         g2o_frontend::Correspondence thecorr=correspondanceVector.at(i);
//         VertexPlane* a=dynamic_cast<VertexPlane*>(thecorr.edge()->vertex(0));
//         VertexPlane* b=dynamic_cast<VertexPlane*>(thecorr.edge()->vertex(1));
// 
//         cout << "MERDGING ["<< a->id()<<"] > ["<< b->id()<<"] result: "<<graph->mergeVertices(a,b,1)<<endl;
//     }
// }

#endif
