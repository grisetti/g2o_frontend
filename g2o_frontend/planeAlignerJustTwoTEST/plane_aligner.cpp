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

#include "fileReading.h"
#include "printHelper.h"
#include "planesTransformations.h"

using namespace std;
using namespace g2o;
using namespace Slam3dAddons;
using namespace cv;
using namespace Eigen;
using namespace g2o_frontend;

VertexPlane myfancyVertex;
RGBDData imageOriginal;

struct planeAndVertex
{
    Plane3D plane;
    VertexSE3 vertex;
    VertexPlane vplane;
};

struct planeCorrespondence
{
    int p1;
    int p2;
    double error;
};

struct error_struct
{
    int v1;
    int v2;
    double error;
};



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
template <typename TypeDomain_, int dimension_>
struct EuclideanMapping{
    typedef TypeDomain_ TypeDomain;
    typedef typename Eigen::Matrix<double, dimension_, 1> VectorType;
    virtual int dimension() const {return dimension_;}
    virtual TypeDomain fromVector(const VectorType& v) const =  0;
    virtual VectorType toVector(const TypeDomain& t) const = 0;
};

template <int dimension_>
struct VectorMapping : public EuclideanMapping<Eigen::Matrix<double, dimension_, 1>, dimension_>{
    typedef typename EuclideanMapping<Eigen::Matrix<double, dimension_, 1>, dimension_>::TypeDomain TypeDomain;
    typedef typename EuclideanMapping<Eigen::Matrix<double, dimension_, 1>, dimension_>::VectorType VectorType;
    virtual TypeDomain fromVector(const VectorType& v) const {return v;}
    virtual VectorType toVector(const TypeDomain& t) const {return t;}
};


struct PlaneMapping: public EuclideanMapping<Plane3D,4>{
    typedef typename EuclideanMapping<Plane3D, 4>::TypeDomain TypeDomain;
    typedef typename EuclideanMapping<Plane3D, 4>::VectorType VectorType;
    virtual TypeDomain fromVector(const VectorType& v) const {
        Plane3D l(v);
        return l;
    }
    virtual VectorType toVector(const TypeDomain& t) const {
        return t.toVector();
    }
};

typedef std::vector<Correspondence> CorrespondenceVector;

template <typename MappingType, typename RansacType, typename EdgeCorrespondenceType>
bool testRansac(typename RansacType::TransformType& result,CorrespondenceVector& correspondences,IndexVector& iv){

    RansacType ransac;

    //ransac.correspondenceValidators()=validators;
    ransac.setCorrespondences(correspondences);
    ransac.setMaxIterations(1000);
    ransac.setInlierErrorThreshold(1.);
    ransac.setInlierStopFraction(0.5);

    return ransac(result,iv);
}

Vector3d aRandColor()
{
    Vector3d theColor((rand()%255)/255,(rand()%255)/255,(rand()%255)/255);
    return theColor;
}


Isometry3d odometry;    //the odometry ISOMETRY
int size=1;             //number of planes to read from the .dat file

int main(int argc, char**argv)
{
    hasToStop = false;
    string filename;
    string outfilename;
    CommandArgs arg;
    int theError;
    int vertex1;

    arg.param("o", outfilename, "otest.g2o", "output file name");
    arg.param("e",theError,5,"errore");
    arg.param("v1",vertex1,1,"primo vertice");
    arg.paramLeftOver("graph-input", filename , "", "graph file which will be processed", true);
    arg.parseArgs(argc, argv);

    //Reading graph
    //**************************************************************************************************************************************
    OptimizableGraph graph;
    OptimizableGraph outgraph;

    signal(SIGINT, sigquit_handler);

    //**************************************************************************************************************************************

    //GENERAZIONE GRAFO ON FLY


    std::ifstream p1("p1.dat");
    std::ifstream transformation("transform.dat");

    //piani non normalizzati da leggere nel file
    Vector4d piano[5];
    //piani non normalizzati transformati, verranno salvati qui
    Vector4d pianoR[5];
    //la trasformata relativa tra i piani, verra' letta dal file e salvata qui
    Isometry3d trasformata;


    fillPlanes("p1.dat",1,piano);
    fillTransform("transform.dat",trasformata);


    //stampe di debug
    cout << "piani nel frame 0"<<endl;
    for(int i =0;i<size;i++)
    {

        printVector4dAsRow(piano[i],1);

    }
    cout << endl;

    cout << "trasformata"<<endl;
    Vector6d trasformataV=g2o::internal::toVectorMQT(trasformata);
    printVector6dAsRow(trasformataV,1);
    cout << endl;

    cout << "piani trasformati nel frame 1"<<endl;
    for(int i=0;i<size;i++)
    {
        //pianoR[i]=remapPlane(piano[i],trasformata);
        pianoR[i]=trasformata*piano[i];
        printVector4dAsRow(pianoR[i],1);
    }
    cout << endl;


    cout << "trasformo in piani in 1 nel sistema di riferimento di 0"<<endl;
    Isometry3d trasformataIsometry2=trasformata.inverse();
    for(int i=0;i<size;i++)
    {
        Vector4d tmp=remapPlane(pianoR[i],trasformata.inverse());
        printVector4dAsRow(tmp,1);
    }
    cout << endl << endl <<"================================================================"<<endl<<endl;

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

    v2->setEstimate(v1->estimate()*v1TOv2->measurement());

    //PARAMETER OFFSET
    VertexSE3* offset=new VertexSE3;
    offset->setId(2545);
    offset->setEstimate(Isometry3d::Identity());
    cout << "Adding offset vertex: ";
    cout <<graph.addVertex(offset)<<endl;

    cout << "Adding vertex: ";
    cout <<graph.addVertex(v1)<<endl;

    cout << "Adding vertex: ";
    cout <<graph.addVertex(v2)<<endl;

    cout << "Adding edge: ";
    cout <<graph.addEdge(v1TOv2)<<endl;

    int planeID=666;
    //VERTICE 1
    for(int i=0;i<size;i++)
    {
        VertexPlane* vPlane=new VertexPlane;
        Plane3D plane;
        plane.fromVector(piano[i]);
        vPlane->setEstimate(plane);
        vPlane->setId(planeID);

        cout << "Adding normalized vertex plane: ";
        cout <<graph.addVertex(vPlane)<<endl;
        cout << plane.toVector()<<endl;

        EdgeSE3PlaneSensorCalib* eSE3calib= new EdgeSE3PlaneSensorCalib;
        eSE3calib->setVertex(0,v1);
        eSE3calib->setVertex(1,vPlane);
        eSE3calib->setVertex(2,offset);
        eSE3calib->color=Vector3d(0,1,0);
        eSE3calib->setMeasurement(plane);

        cout <<"Adding edge plane: ";
        cout <<graph.addEdge(eSE3calib)<<endl;
        planeID++;
    }
    //VERTICE 2

    for(int i=0;i<size;i++)
    {
        VertexPlane* vPlane=new VertexPlane;
        Plane3D plane;
        plane.fromVector(pianoR[i]);
        vPlane->setEstimate(plane);
        vPlane->setId(planeID);

        cout << "Adding normalized vertex plane: ";
        cout <<graph.addVertex(vPlane)<<endl;
        cout << plane.toVector()<<endl;

        EdgeSE3PlaneSensorCalib* eSE3calib= new EdgeSE3PlaneSensorCalib;
        eSE3calib->setVertex(0,v2);
        eSE3calib->setVertex(1,vPlane);
        eSE3calib->setVertex(2,offset);

        eSE3calib->setMeasurement(v2->estimate().inverse()*plane);
        eSE3calib->color=Vector3d(1,0,0);
        cout <<"Adding edge plane: ";
        cout <<graph.addEdge(eSE3calib)<<endl;
        planeID++;
    }


    cout << "salvo grafo intermedio...";
    ofstream grafino ("grafene.g2o");
    graph.save(grafino);
    cout << "salvato"<<endl;


    Isometry3d transformation2to1;
    transformation2to1.setIdentity();
    Isometry3d cameraOffset;
    cameraOffset.setIdentity();
    CorrespondenceVector mycorrVector;

    //recupero il vertice richiesto in input
    OptimizableGraph::Vertex* _v=graph.vertex(0);
    _v->setFixed(true);
    VertexSE3* v=dynamic_cast<  VertexSE3*>(_v);
    VertexSE3* vnext=new VertexSE3;
    VertexSE3* a=new VertexSE3;
    VertexSE3* sensorOffset=new VertexSE3;

    std::vector<int> verticesToBeSaved;


    //v è un vertex SE3
    if(v)
    {

        //accedo ad ogni edge di quel vertice
        OptimizableGraph::EdgeSet e = v->edges();

        //di quei edge accedo ad ogni vertice successivo
        for (HyperGraph::EdgeSet::iterator it = e.begin(); it!=e.end(); it++)
        {

            HyperGraph::Edge* _e = *it;
            //accedo solo a quelli SE3 per sapere su quale vertice spostarmi
            EdgeSE3 * eSE3 =dynamic_cast< EdgeSE3*>(_e);

            if(eSE3)
            {
                //accedo al vertice successivo sul quale andare
                a = dynamic_cast<  VertexSE3*>(eSE3->vertex(1));

                //VertexSE3* b = dynamic_cast<  VertexSE3*>(eSE3->vertex(0));


                if(a->id()!=v->id() && a)
                {
                    cerr <<"V[" <<v->id() << "] -> V[" << a->id() <<"] "<< endl;


                    _v=graph.vertex(a->id());
                    odometry=eSE3->measurement();
                    cout << "ODO:"<<endl;
                    cout << g2o::internal::toVectorMQT(odometry);
                    cout << endl;

                    outgraph.addEdge(eSE3);
                    //_v è il nuovo vertice su cui mi devo spostare
                    vnext=dynamic_cast<VertexSE3*>(_v);
                }
            }
        }
    }

    //a questo punto:
    //  v       vertice iniziale
    //  vnext   vertice successivo

    OptimizableGraph::EdgeSet v_EDGES       = v->edges();
    OptimizableGraph::EdgeSet v_next_EDGES  = vnext->edges();

    outgraph.addVertex(v);
    outgraph.addVertex(vnext);

    vector<VertexPlane*> plane_v_container;
    vector<VertexPlane*> plane_v_next_container;
    vector<Plane3D> REMAPPED_plane_v_next_container;



    for (HyperGraph::EdgeSet::iterator it = v_EDGES.begin(); it!=v_EDGES.end(); it++)
    {
        HyperGraph::Edge* _e = *it;
        EdgeSE3PlaneSensorCalib * eSE3Calib = new EdgeSE3PlaneSensorCalib;
        eSE3Calib   =dynamic_cast< EdgeSE3PlaneSensorCalib*>(_e);
        Matrix3d info= Matrix3d::Identity();
        info*=1000;
        info(2,2)=10; //il vincolo in traslazione avrà un peso minore

        if(eSE3Calib)
        {




            eSE3Calib->setInformation(info);
            VertexPlane* vplane=new VertexPlane;
            vplane->setEstimate(eSE3Calib->measurement());

            eSE3Calib->color=Vector3d(1.0,0.1,0.1);  //PRIMO FRAME ROSSO
            outgraph.addEdge(eSE3Calib);
            if(vplane)
            {
                vplane->color=Vector3d(0,0,0);
                Plane3D plane = vplane->estimate();
                plane_v_container.push_back(vplane);
            }
        }
    }

    Matrix3d info= Matrix3d::Identity();

    for (HyperGraph::EdgeSet::iterator it = v_next_EDGES.begin(); it!=v_next_EDGES.end(); it++)
    {
        HyperGraph::Edge* _e = *it;
        EdgeSE3PlaneSensorCalib * eSE3Calib = new EdgeSE3PlaneSensorCalib;
        eSE3Calib   =dynamic_cast< EdgeSE3PlaneSensorCalib*>(_e);

        info(0,0)==1000;
        info(1,1)=1000;
        info(2,2)=10; //il vincolo in traslazione avrà un meso minore

        if(eSE3Calib)
        {



            eSE3Calib->setInformation(info);
            sensorOffset = dynamic_cast< VertexSE3*>(eSE3Calib->vertex(2));
            VertexPlane* vplane = dynamic_cast< VertexPlane*>(eSE3Calib->vertex(1));
            eSE3Calib->color=Vector3d(0.1,1.0,0.1); //SECONDO FRAME VERDE
            outgraph.addEdge(eSE3Calib);

            Plane3D piano=eSE3Calib->measurement();
            VertexPlane * vpiano = new VertexPlane;
            vpiano->setEstimate(piano);
            plane_v_next_container.push_back(vpiano);

            cout << "[VNEXT] "<<endl;
            cout << eSE3Calib->measurement().toVector()<<endl;

        }
    }



    cout << "Il primo   container è composta da " <<plane_v_container.size()<<endl;
    cout << "Il secondo container è composta da " <<plane_v_next_container.size()<<endl;

    //remapping things

    for(int ik=0;ik<plane_v_next_container.size();ik++ )
    {
        //odometry.setIdentity();

        Plane3D modifiedPlaneNext;
        Plane3D planenext=(plane_v_next_container.at(ik))->estimate();
        modifiedPlaneNext=odometry*planenext;

        //REMAPPED_plane_v_next_container.push_back(modifiedPlaneNext);

        VertexPlane* tmpV=plane_v_container.at(ik);
        Plane3D tmpP=tmpV->estimate();



        cout <<"ODO "<<endl << g2o::internal::toVectorMQT(odometry)<<endl;

        cout <<"Frame 0"<<endl<<tmpP.toVector()<<endl<<endl;
        cout <<"Frame 1"<<endl<<planenext.toVector()<<endl<<endl;

        cout<< "Frame 1 trasformato in 0"<< endl<<modifiedPlaneNext.toVector()<<endl<<endl;

        cout <<"Frame 0 trasformato in 1"<<endl<<(odometry.inverse()*tmpP).toVector()<<endl<<endl;

    }


    int incrementer=9999;
    for(int ik=0;ik<REMAPPED_plane_v_next_container.size();ik++)
    {
        VertexPlane* remapped_V = new VertexPlane();
        EdgeSE3PlaneSensorCalib* remapped_E= new EdgeSE3PlaneSensorCalib();
        remapped_V->setId(incrementer);
        remapped_V->setEstimate(REMAPPED_plane_v_next_container.at(ik));
        remapped_E->setMeasurement(REMAPPED_plane_v_next_container.at(ik));
        remapped_E->setVertex(0,v);
        remapped_E->setVertex(1,remapped_V);
        remapped_E->setVertex(2,sensorOffset);

        remapped_E->color=Vector3d(0,0,1);
        remapped_V->color=Vector3d(0,0,1);

        remapped_E->setInformation(info);
        outgraph.addVertex(remapped_V);
        outgraph.addEdge(remapped_E);

        cout << "Aggiungo nuovo vertice al grafo...["<< incrementer <<"]"<<endl;
        incrementer++;
    }


    cout << "Salvataggio nel grafo di output..."<<endl<<endl;
    //rimuovo il fix dal vertice 2545
    OptimizableGraph::Vertex* tmpVertex=graph.vertex(2545);
    if(tmpVertex)
    {
        tmpVertex->setFixed(false);
        cout << "param vertex found and set"<<endl;
    }
    else cout << "param vertex not found"<<endl;

    //ofstream grafino ("grafino.g2o");
    //outgraph.save(grafino);

    cout <<"--------------------------------------------------"<<endl;
    cout <<"Computazione Errore"<<endl;



    //Creazione del vettore delle corrispondenze e calcolo degli errori.

    for(int ei=0;ei<plane_v_container.size();ei++)
    {
        VertexPlane* first_frame_vertex_plane=new VertexPlane();
        first_frame_vertex_plane=plane_v_container.at(ei);
        Plane3D first_frame_plane=first_frame_vertex_plane->estimate();
        //cout << "outer "<<ei<<endl;
        for(int ri=0;ri<REMAPPED_plane_v_next_container.size();ri++)
        {

            //cout << "inner "<<ri<<endl;
            Plane3D remapped_frame_plane=REMAPPED_plane_v_next_container.at(ri);
            Vector4d diff = first_frame_plane.toVector()-remapped_frame_plane.toVector();
            //diff.head<3>() *= 100;
            double error = diff.squaredNorm();
            cout <<first_frame_plane.toVector()[0]<<","
                <<first_frame_plane.toVector()[1]<<","
               <<first_frame_plane.toVector()[2]<<","
              <<first_frame_plane.toVector()[3]<<","
             <<"="
            <<remapped_frame_plane.toVector()[0]<<","
            <<remapped_frame_plane.toVector()[1]<<","
            <<remapped_frame_plane.toVector()[2]<<","
            <<remapped_frame_plane.toVector()[3];

            cout <<"["<< error <<"]"<< endl;

            EdgePlane* eplane = new EdgePlane;
            eplane->setVertex(0,first_frame_vertex_plane);
            eplane->setVertex(1,plane_v_next_container.at(ri));
            Correspondence corr(eplane,error);
            mycorrVector.push_back(corr);

        }
        cout << endl;
    }


//    Isometry3d tresult;
//    tresult.setIdentity();
//    IndexVector iv;
//    RansacPlaneLinear ransac;

//    ransac.setCorrespondences(mycorrVector);
//    ransac.setMaxIterations(2000);
//    ransac.setInlierErrorThreshold(0.9);
//    ransac.setInlierStopFraction(0.5);

//    ransac(tresult,iv);

//    bool result = testRansac<PlaneMapping, RansacPlaneLinear, EdgePlane>(tresult, mycorrVector,iv);

//    Vector6d ttt=g2o::internal::toVectorMQT(tresult);
//    Vector6d ttt2=g2o::internal::toVectorMQT(tresult.inverse());
//    cerr << "Transformation result from ransac"<<endl<<ttt<<endl;
//    cerr << "Transformation result from ransac"<<endl<<ttt2<<endl;
//    cerr << "Odometry from robot"<<endl<<g2o::internal::toVectorMQT(odometry)<<endl;

    exit(0);
}

