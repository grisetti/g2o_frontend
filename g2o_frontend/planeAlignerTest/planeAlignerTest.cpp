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
#include "g2o_frontend/data/point_cloud_data.h"
#include "g2o_frontend/sensor_data/rgbd_data.h"
#include <Eigen/Geometry>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include "g2o/types/slam3d_addons/vertex_plane.h"
#include "g2o/types/slam3d_addons/edge_plane.h"
#include "g2o/types/slam3d_addons/edge_se3_plane_calib.h"
#include "g2o/core/hyper_graph.h"
#include "g2o_frontend/ransac/ransac.h"
#include "g2o_frontend/ransac/alignment_plane_linear.h"
#include "g2o_frontend/basemath/bm_se2.h"
#include "g2o/types/slam3d/isometry3d_mappings.h"
#include <iostream>
#include <fstream>
#include <cstdlib>
#include "g2o_frontend/ransac/alignment_plane_linear.h"
#include "g2o_frontend/ransac/ransac.h"
#include <cstdio>
#include <iomanip>
#include <stdio.h>
#include <iostream>
#include "fileReading.h"
#include "printHelper.h"
#include "planesTransformations.h"
#include "graphUtils.h"
#include "ransacDeclarations.h"

using namespace std;
using namespace g2o;
using namespace Slam3dAddons;
using namespace cv;
using namespace Eigen;
using namespace g2o_frontend;


volatile bool hasToStop;

//Signal Handling
//**************************************************************************************************************************************
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
//**************************************************************************************************************************************


Isometry3d odometry;    //the odometry ISOMETRY


int main(int argc, char**argv)
{
    cout << endl << endl <<"================================================================"<<endl<<endl;

    //Globals
    //**************************************************************************************************************************************
    hasToStop = false;
    string filename;
    CommandArgs arg;
    int vertex1;
    int parameterID;
    double errorREF;
    //**************************************************************************************************************************************

    //Inits arguments
    //**************************************************************************************************************************************
    arg.param("v1",vertex1,0,"primo vertice");
    arg.param("e",errorREF,1,"errore per il merging");
    arg.param("p",parameterID,30,"id del vertice del parametro");
    arg.paramLeftOver("graph-input", filename , "", "graph file which will be processed", true);
    arg.parseArgs(argc, argv);
    //**************************************************************************************************************************************

    //Reading graph
    //**************************************************************************************************************************************
    OptimizableGraph graph;
    VertexSE3* v1;
    VertexSE3* v2;
    //**************************************************************************************************************************************

    //Stuff
    //**************************************************************************************************************************************
    signal(SIGINT, sigquit_handler);
    //**************************************************************************************************************************************


    //Ransac Things
    //**************************************************************************************************************************************
    CorrespondenceVector mycorrVector;
    Matrix3d info= Matrix3d::Identity();
    info*=1000;
    info(2,2)=10; //il vincolo in traslazione avrà un peso minore
    //**************************************************************************************************************************************

    cout << "Loading graph file...";
    graph.load(filename.c_str());
    cout << "done"<<endl;

    //recupero il vertice richiesto in input
    OptimizableGraph::Vertex* _v=graph.vertex(vertex1);
    _v->setFixed(true);

    v1=dynamic_cast<VertexSE3*>(_v);
    EdgeSE3 * eSE3=new EdgeSE3;

    if(v1)
    {
        get_next_vertexSE3(&graph,v1,v2,odometry,eSE3);
        graph.addEdge(eSE3);
    }

    //a questo punto:
    //  v1   vertice iniziale
    //  v2   vertice successivo

    cout<<endl;
    graph.addVertex(v1);
    graph.addVertex(v2);
    cout<<endl;



    vector<container> plane_1_container;
    vector<container> plane_2_container;
    vector<container> plane_2_container_REMAPPED;



    //cout << "V1"<<endl;
    getCalibPlanes(v1,&plane_1_container,Vector3d(1,0,0),info);
    //cout << "V2"<<endl;
    getCalibPlanes(v2,&plane_2_container,Vector3d(0,1,0),info);





    //--------------------------INIZIO DEBUG
    cout << "Il primo   container è composta da " <<plane_1_container.size()<<" elemento"<< endl;

    for(unsigned int i=0;i<plane_1_container.size();i++)
    {
        graph.addVertex(plane_1_container.at(i).plane);
        Plane3D tmpPlane=((plane_1_container.at(i)).plane)->estimate();
        printPlaneCoeffsAsRow(tmpPlane,1);
    }

    cout << "Il secondo container è composta da " <<plane_2_container.size()<<" elemento"<<endl;

    for(unsigned int i=0;i<plane_2_container.size();i++)
    {
        graph.addVertex(plane_2_container.at(i).plane);
        Plane3D tmpPlane=((plane_2_container.at(i).plane))->estimate();
        printPlaneCoeffsAsRow(tmpPlane,1);
    }



    cout << "Il secondo container di piani rimappati" <<endl;

    for(unsigned int i=0;i<plane_2_container.size();i++)
    {


        Plane3D tmpPlane=((plane_2_container.at(i)).plane)->estimate();
        tmpPlane=odometry*tmpPlane;
        container c;

        c.id=(plane_2_container.at(i)).id;

        c.plane=new VertexPlane;
        c.plane->setEstimate(tmpPlane);

        plane_2_container_REMAPPED.push_back(c);

        printPlaneCoeffsAsRow(tmpPlane,1);

    }


    //--------------------------FINE DEBUG

    cout <<"--------------------------------------------------"<<endl;
    cout <<"--------------------------------------------------"<<endl;
    cout <<"Computazione Errore"<<endl;
    cout <<"--------------------------------------------------"<<endl;
    cout <<"--------------------------------------------------"<<endl;


    //Creazione del vettore delle corrispondenze e calcolo degli errori.
    compute_Correspondance_Vector(plane_1_container,plane_2_container,plane_2_container_REMAPPED,mycorrVector,errorREF);



    cout << endl << " ~~~~ CHECK ~~~~"<<endl<<endl;
    for(unsigned int i=0;i<mycorrVector.size();i++)
    {

        g2o_frontend::Correspondence c= mycorrVector.at(i);
        EdgePlane* eplane;
        eplane=dynamic_cast<EdgePlane*>(c.edge());
        VertexPlane* vp1=dynamic_cast<VertexPlane*>(eplane->vertex(0));
        VertexPlane* vp2=dynamic_cast<VertexPlane*>(eplane->vertex(1));

        Plane3D pp1=vp1->estimate();
        Plane3D pp2=vp2->estimate();

        cout <<i <<" " <<c.score()<<" "<<pp1.coeffs().transpose()<< "  " << pp2.coeffs().transpose()<<endl;

    }


    //***************************************************************************************************************************

    Isometry3d tresult;
    IndexVector iv;

    executeRansac(mycorrVector,iv,tresult,100000,0.02,0.5);

    //--------------------------INIZIO DEBUG
    Vector6d result_DIRECT=g2o::internal::toVectorMQT(tresult);
    Vector6d result_INVERSE=g2o::internal::toVectorMQT(tresult.inverse());
    Vector6d ground_truth=g2o::internal::toVectorMQT(odometry);

    cerr << "Transformation result from ransac"<<endl;
    printVector6dAsRow(result_DIRECT,1);
    cout << endl;
    cerr << "Transformation result (inverse) from ransac"<<endl;
    printVector6dAsRow(result_INVERSE,1);
    cout << endl;

    Vector6d error=g2o::internal::toVectorMQT(odometry*tresult.inverse());
    cerr<< "MPLY transformations..."<<endl;
    cerr <<error.transpose() <<endl;
    cerr<< "MPLY..."<<endl;
    cerr <<error.squaredNorm() <<endl;



    cerr << "Odometry from robot"<<endl;
    printVector6dAsRow(ground_truth,1);
    cout << endl;

    cout << "SIZE INLIERS "<<iv.size()<<endl;

    //    cout << "salvo grafo intermedio...";
    //    ofstream notmerged ("notmerged.g2o");
    //    graph.save(notmerged);
    //    cout << "salvato"<<endl;

    std::set<int> vertexPlaneIds;

    cout << "Merging vertices...";
    for(unsigned int i =0;i<mycorrVector.size();i++)
    {
        Correspondence corr=mycorrVector.at(i);
        VertexPlane* v1=dynamic_cast<VertexPlane*>(corr.edge()->vertex(0));
        VertexPlane* v2=dynamic_cast<VertexPlane*>(corr.edge()->vertex(1));
        graph.mergeVertices(v1,v2,0);
        vertexPlaneIds.insert(v1->id());
    }
    cout << "done"<<endl;


    EdgeSE3* anotherEdge=new EdgeSE3;
    anotherEdge->setVertex(0,v1);
    anotherEdge->setVertex(1,v2);
    anotherEdge->setMeasurement(tresult);

    graph.addEdge(anotherEdge);

    //RIMUOVERE ROBA INUTILE


    for (OptimizableGraph::VertexIDMap::iterator it = graph.vertices().begin(); it!=graph.vertices().end(); it++)
    {
        VertexSE3* vs3= dynamic_cast<VertexSE3*>(it->second);
        if(vs3)
        {
            if( !(vs3->id()==parameterID || vs3->id()==v1->id() || vs3->id()==v2->id()) )
            {
                //cout << "DELETING VERTEX "<<vs3->id()<<endl;
                graph.removeVertex(vs3,1);
            }
        }
        else
        {
            VertexPlane* vp=dynamic_cast<VertexPlane*>(it->second);
            if(vp)
            {
                std::set<int>::iterator it= vertexPlaneIds.find(vp->id());
                cout <<endl<< "check "<<vp->id()<< " ";
                if(it==vertexPlaneIds.end())
                {
                    cout << "deleting!"<<endl;
                    graph.removeVertex(vp,1);
                }
            }
        }

    }


    cout << "salvo grafo finale...";
    ofstream merged ("merged.g2o");
    graph.save(merged);
    cout << "salvato"<<endl;
}

