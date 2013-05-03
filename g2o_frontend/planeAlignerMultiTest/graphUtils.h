#ifndef _MAL_GRAPHUTILS
#define _MAL_GRAPHUTILS


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
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"
#include "g2o_frontend/data/point_cloud_data.h"
#include "g2o_frontend/sensor_data/rgbd_data.h"

#include "ransacDeclarations.h"
#include "printHelper.h"

using namespace Eigen;
using namespace g2o;
using namespace Slam3dAddons;

struct container{
    int id;
    VertexPlane* plane;
};


void getCalibPlanes(VertexSE3* v,vector<container>* containerP,Vector3d color,Matrix3d &info)
{
    OptimizableGraph::EdgeSet edges  = v->edges();

    for (HyperGraph::EdgeSet::iterator it = edges.begin(); it!=edges.end(); it++)
    {
        HyperGraph::Edge* _e = *it;
        EdgeSE3PlaneSensorCalib * eSE3Calib = new EdgeSE3PlaneSensorCalib;
        eSE3Calib   =dynamic_cast< EdgeSE3PlaneSensorCalib*>(_e);

        if(eSE3Calib)
        {
            eSE3Calib->setInformation(info);
            VertexPlane* vplane=dynamic_cast<VertexPlane*>(eSE3Calib->vertex(1));
            eSE3Calib->color=color;
            if(vplane)
            {

                vplane->color=color;
                container c;
                c.id=vplane->id();
                c.plane=vplane;
                containerP->push_back(c);
            }
        }
    }
}


double computeError(Plane3D &p1,Plane3D &p2,double e1,double e2)
{
    Vector4d diff = p1.toVector()-p2.toVector();
    diff.head<3>() *= e1;
    diff(4) *= e2;
//    diff.head<3>() *= 10;
//    diff(4) *= 1;
    double Error=diff.squaredNorm();
    return Error;
}

//INPUT
//  the graph
//  the initial vertex
//OUTPUT
//  the next vertex in the odometry
void get_next_vertexSE3(OptimizableGraph* graph,VertexSE3* v1, VertexSE3* & v2,Isometry3d &odometry,EdgeSE3* eSE3)
{
    OptimizableGraph::Vertex* _vTEMP;

    //accedo ad ogni edge di quel vertice
    OptimizableGraph::EdgeSet e = v1->edges();
    //di quei edge accedo ad ogni vertice successivo
    for (HyperGraph::EdgeSet::iterator it = e.begin(); it!=e.end(); it++)
    {

        HyperGraph::Edge* _e = *it;
        //accedo solo a quelli SE3 per sapere su quale vertice spostarmi
        eSE3 =dynamic_cast< EdgeSE3*>(_e);

        if(eSE3)
        {

            Matrix6d eseINFO;
            eseINFO.setZero();
            eseINFO(0,0)=100;
            eseINFO(1,1)=100;
            eseINFO(2,2)=10000;
            eseINFO(3,3)=1000;
            eseINFO(4,4)=1000;
            eseINFO(5,6)=1000;
            //eSE3->setInformation(eseINFO);

            //accedo al vertice successivo sul quale andare
            VertexSE3* nextVertex = dynamic_cast<  VertexSE3*>(eSE3->vertex(1));

            //verifico che il vertice che ho recuperato dall'Edge è effettivamente
            //- di tipo EdgeSE3 (verificato in precedenza)
            //- il suo id è differente da quello di partenza

            if(nextVertex->id()!=v1->id() && nextVertex)
            {
                cout << "mi muovo da - a"<<endl;
                cout <<"V[" <<v1->id() << "] -> V[" << nextVertex->id() <<"] "<< endl;
                _vTEMP=graph->vertex(nextVertex->id());

                //se va tutto bene a questo punto l'odometria deve essere la stessa della trasformata iniziale
                odometry=eSE3->measurement();
                cout << "Odometria letta dal grafo:"<<endl;
                printVector6dAsRow(g2o::internal::toVectorMQT(odometry),1);
                //cout << "Trasformata letta da file:"<<endl;
                //printVector6dAsRow(g2o::internal::toVectorMQT(trasformata),1);

                //outgraph.addEdge(eSE3);
                //_v è il nuovo vertice su cui mi devo spostare
                v2=dynamic_cast<VertexSE3*>(_vTEMP);
            }
        }
    }
}



void compute_Correspondance_Vector(vector<container> &c1,
                                   vector<container> &c2,
                                   vector<container> &c2R,
                                   CorrespondenceVector &correspondanceVector, double THEerrorREF,double e1, double e2)
{
    // C1 plane container 1
    // C2 plane container 2
    // C2R plane container 2

    for(unsigned int i=0;i<c1.size();i++)
    {

        Plane3D p1=((c1.at(i)).plane)->estimate();

        for(unsigned int j=0;j<c2.size();j++)
        {

            Plane3D p2=((c2.at(j)).plane)->estimate();
            Plane3D p2R=((c2R.at(j)).plane)->estimate();

            double error = computeError(p1,p2R,e1,e2);

            printPlaneCoeffsAsRow(p1);
            cout << " <> ";
            printPlaneCoeffsAsRow(p2);
            cout <<" ["<<error<<"]"<<endl;
            //FILLING CORRESPONDANCE VECTOR
            EdgePlane* eplane = new EdgePlane;

            eplane->setVertex(0,c1.at(i).plane);
            eplane->setVertex(1,c2.at(j).plane);
            g2o_frontend::Correspondence corr(eplane,error);

            if(error<THEerrorREF)
            {

            correspondanceVector.push_back(corr);
            }

        }

    }
}

void executeRansac(const CorrespondenceVector &correspondanceVector,
                   std::vector<int> &Indeces,
                   Isometry3d &transform,
                   int iterations,
                   float inliersThreshold,
                   float inliersStop)
{

    transform.setIdentity();

    g2o_frontend::RansacPlaneLinear ransac;
    ransac.setCorrespondences(correspondanceVector);
    ransac.setMaxIterations(iterations);
    ransac.setInlierErrorThreshold(inliersThreshold);
    ransac.setInlierStopFraction(inliersStop);
    cout << "RESULT "<< ransac(transform,Indeces,1)<<endl;
}

void merge_vertices(OptimizableGraph* graph,CorrespondenceVector &correspondanceVector)
{
    for(unsigned int i =0;i<correspondanceVector.size();i++)
    {
        g2o_frontend::Correspondence thecorr=correspondanceVector.at(i);
        VertexPlane* a=dynamic_cast<VertexPlane*>(thecorr.edge()->vertex(0));
        VertexPlane* b=dynamic_cast<VertexPlane*>(thecorr.edge()->vertex(1));

        cout << "MERDGING ["<< a->id()<<"] > ["<< b->id()<<"] result: "<<graph->mergeVertices(a,b,1)<<endl;
    }
}

#endif
