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
    string outfilename;
    CommandArgs arg;
    int theError;
    int vertex1;
    int ransac;
    int size;       //number of planes to read from the .dat file
    //**************************************************************************************************************************************

    //Inits arguments
    //**************************************************************************************************************************************
    arg.param("o", outfilename, "otest.g2o", "output file name");
    arg.param("e",theError,5,"errore");
    arg.param("r",ransac,0,"ransac");
    arg.param("s",size,1,"size");
    arg.param("v1",vertex1,1,"primo vertice");
    arg.paramLeftOver("graph-input", filename , "", "graph file which will be processed", true);
    arg.parseArgs(argc, argv);
    //**************************************************************************************************************************************

    //Reading graph
    //**************************************************************************************************************************************
    OptimizableGraph graph;
    OptimizableGraph outgraph;
    //**************************************************************************************************************************************

    //Stuff
    //**************************************************************************************************************************************
    signal(SIGINT, sigquit_handler);
    //**************************************************************************************************************************************


    //Ransac Things
    //**************************************************************************************************************************************
    CorrespondenceVector mycorrVector;
    //**************************************************************************************************************************************

    //GENERAZIONE GRAFO ON FLY
    //  leggo da due file
    //- p1.dat contiene i piani nel frame 0
    //- p2.dat contiene i piani nel frame 1
    //- transform.dat contiene la trasformata relativa tra i due

    //piani non normalizzati da leggere nel file
    Vektor piano;
    //piani non normalizzati transformati, verranno salvati qui
    Vektor pianoR;
    //la trasformata relativa tra i piani, verra' letta dal file e salvata qui
    Isometry3d trasformata;

    char file1[] = "p1.dat";
    char file2[] = "p2.dat";
    char file3[] = "transform.dat";

    fillPlanes(file1,size,piano);
    fillPlanes(file2,size,pianoR);
    fillTransform(file3,trasformata);

    cout << "trasformata"<<endl;
    Vector6d trasformataV=g2o::internal::toVectorMQT(trasformata);
    printVector6dAsRow(trasformataV,1);

    cout << trasformata.matrix()<<endl;
    cout << endl;

    //stampe di debug
    cout << "piani nel frame 0"<<endl;
    for(int i =0;i<size;i++)
    {
        printVector4dAsRow(piano[i],1);
    }
    cout << endl;

    cout << "piani nel frame 1"<<endl;
    for(int i=0;i<size;i++)
    {
        cout << pianoR[i].transpose()<<endl;
    }
    cout << endl;


    cout << "piani rimappati da 0 a 1"<<endl;
    for(int i=0;i<size;i++)
    {

        Isometry3d t=trasformata.inverse();
        Plane3D p;
        p.fromVector(piano[i]);
        cout << piano[i].transpose() <<" -> "<<(t*p).coeffs().transpose()<<endl;
    }

    cout << "piani rimappati da 1 a 0"<<endl;
    for(int i=0;i<size;i++)
    {

        Plane3D p;
        p.fromVector(pianoR[i]);
        cout << pianoR[i].transpose() <<" -> "<<(trasformata*p).coeffs().transpose()<<endl;
    }

    cout << endl;

    cout << endl << endl <<"================================================================"<<endl<<endl;

    //************************************************************************************************
    //************************************************************************************************

    VertexSE3* v1 = new VertexSE3;
    VertexSE3* v2 = new VertexSE3;

    v1->setId(0);
    v2->setId(1);

    Isometry3d v1Est;
    v1Est.setIdentity();
    v1->setEstimate(v1Est);

    EdgeSE3* v1TOv2=new EdgeSE3;
    v1TOv2->setMeasurement(trasformata);
    v1TOv2->setVertex(0,v1);
    v1TOv2->setVertex(1,v2);

    v2->setEstimate(v1->estimate()*v1TOv2->measurement()); // v1->estimate()*v1TOv2->measurement() ====>>> LA TRASFORMATA

    //PARAMETER OFFSET
    VertexSE3* offset=new VertexSE3;
    offset->setId(2545);
    offset->setEstimate(Isometry3d::Identity());
    offset->setFixed(1);
    graph.addVertex(offset);

    v1->setFixed(1);
    v2->setFixed(1);

    graph.addVertex(v1);
    graph.addVertex(v2);
    graph.addEdge(v1TOv2);

    int planeID=666;

    Matrix3d info= Matrix3d::Identity();
    info*=1000;
    info(2,2)=10; //il vincolo in traslazione avrà un peso minore


    //VERTICE 1
    for(int i=0;i<size;i++)
    {
        VertexPlane* vPlane=new VertexPlane;
        Plane3D plane;
        plane.fromVector(piano[i]);
        vPlane->setEstimate(plane);
        vPlane->setId(planeID);
        vPlane->color=Vector3d(0.3,0.7,0.3);
        cout << "[1]"<<endl<<"Adding normalized vertex plane: ";
        graph.addVertex(vPlane);
        printPlaneCoeffsAsRow(plane,1);

        EdgeSE3PlaneSensorCalib* eSE3calib= new EdgeSE3PlaneSensorCalib;
        eSE3calib->setVertex(0,v1);
        eSE3calib->setVertex(1,vPlane);
        eSE3calib->setVertex(2,offset);
        eSE3calib->color=Vector3d(0,1,0);
        eSE3calib->setMeasurement(plane);

        eSE3calib->setInformation(info);

        cout <<"Adding edge plane "<<plane.coeffs().transpose() <<endl<<endl;
        graph.addEdge(eSE3calib);
        planeID++;
    }
    //VERTICE 2

    for(int i=0;i<size;i++)
    {
        VertexPlane* vPlane=new VertexPlane;
        Plane3D plane;
        plane.fromVector(piano[i]);
        vPlane->setEstimate(plane);
        vPlane->setId(planeID);
        vPlane->color=Vector3d(0.7,0.3,0.3);
        cout << "[2]"<<endl<<"Adding normalized vertex plane: ";
        graph.addVertex(vPlane);
        printPlaneCoeffsAsRow(plane,1);
        EdgeSE3PlaneSensorCalib* eSE3calib= new EdgeSE3PlaneSensorCalib;
        eSE3calib->setVertex(0,v2);
        eSE3calib->setVertex(1,vPlane);
        eSE3calib->setVertex(2,offset);
        eSE3calib->setInformation(info);
        //--------------------------------> IMPORTANT <-----------------------------------------
        eSE3calib->setMeasurement(v2->estimate().inverse()*plane);
        //eSE3calib->setMeasurement(plane);
        //--------------------------------> IMPORTANT <-----------------------------------------

        eSE3calib->color=Vector3d(1,0,0);
        cout <<"Adding edge plane "<<(eSE3calib->measurement()).coeffs().transpose() <<endl<<endl;
        graph.addEdge(eSE3calib);

        planeID++;
    }


    cout << "salvo grafo intermedio...";
    ofstream saver (outfilename.c_str());
    graph.save(saver);
    cout << "salvato"<<endl;

    cout << endl <<"================================================================"<<endl<<endl;
    exit(0);
}

