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

std::vector<planeAndVertex> container1;
std::vector<planeAndVertex> container2;
std::vector<planeCorrespondence> corrs;

std::set<VertexPlane*> morte;

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


Isometry3d odometry;


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


    cout << "<Parsing "<< filename <<" file>"<<endl;
    graph.load(filename.c_str());
    cout << "<done>"<<endl;

    cerr << "loaded graph, now processing planes" << endl;
    signal(SIGINT, sigquit_handler);

    //**************************************************************************************************************************************


    cerr <<"entering cycle..."<<endl;
    Isometry3d transformation2to1;
    transformation2to1.setIdentity();
    Isometry3d cameraOffset;
    cameraOffset.setIdentity();
    CorrespondenceVector mycorrVector;

    //recupero il vertice richiesto in input
    OptimizableGraph::Vertex* _v=graph.vertex(vertex1);
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
                    cout << odometry.translation();
                    cout << endl;
                    cout << odometry.rotation();
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



    vector<error_struct> error_container;


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

    //    for(int ok=0;ok<plane_v_next_container.size();ok++)
    //    {
    //        VertexPlane * vpiano;
    //        vpiano= plane_v_next_container.at(ok);
    //        cout <<"----------"<<endl;
    //        cout <<vpiano->estimate().toVector();
    //        cout <<"----------"<<endl;
    //    }

    cout << "Il primo   container è composta da " <<plane_v_container.size()<<endl;
    cout << "Il secondo container è composta da " <<plane_v_next_container.size()<<endl;
    cout << "Odometry "<< odometry.translation()<<endl;
    //remapping things

    for(int ik=0;ik<plane_v_next_container.size();ik++ )
    {
        Plane3D modifiedPlaneNext;
        Plane3D planenext=(plane_v_next_container.at(ik))->estimate();
        modifiedPlaneNext=sensorOffset->estimate().inverse()*odometry*sensorOffset->estimate()*planenext;
        REMAPPED_plane_v_next_container.push_back(modifiedPlaneNext);

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

    ofstream grafino ("grafino.g2o");
    outgraph.save(grafino);

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


    Isometry3d tresult;
    tresult.setIdentity();
    IndexVector iv;
    RansacPlaneLinear ransac;

    ransac.setCorrespondences(mycorrVector);
    ransac.setMaxIterations(2000);
    ransac.setInlierErrorThreshold(0.9);
    ransac.setInlierStopFraction(0.5);

    ransac(tresult,iv);

    bool result = testRansac<PlaneMapping, RansacPlaneLinear, EdgePlane>(tresult, mycorrVector,iv);

    cerr << "Transformation result from ransac"<<endl<<g2o::internal::toVectorMQT(tresult)<<endl;
    cerr << "Odometry from robot"<<endl<<g2o::internal::toVectorMQT(odometry)<<endl;





    //                    //adesso mi serve cercare tutti i piani che sono legati all'edge su cui sto lavorando
    //                    for (HyperGraph::EdgeSet::iterator it = e.begin(); it!=e.end(); it++)
    //                    {
    //                        HyperGraph::Edge* _e = *it;
    //                        EdgeSE3PlaneSensorCalib * eSE3Calib = new EdgeSE3PlaneSensorCalib;
    //                        eSE3Calib   =dynamic_cast< EdgeSE3PlaneSensorCalib*>(_e);

    //                        Matrix3d info= Matrix3d::Identity();
    //                        info*=1000;
    //                        info(2,2)=10;


    //                        //elaboro ogni EDGE SE3 CALIB di quel determinato vertice
    //                        if(eSE3Calib)
    //                        {
    //                            //fisso la l'infomation matrix
    //                            eSE3Calib->setInformation(info);
    //                            VertexPlane* vplane = dynamic_cast< VertexPlane*>(eSE3Calib->vertex(1));
    //                            vplane->color=Vector3d(0,0,0);
    //                            eSE3Calib->color=Vector3d(0.1,0.1,0.1);

    //                            //aggiungo un vertice tra i due

    //                            if(vplane)
    //                            {
    //                                Plane3D plane = vplane->estimate();

    //                                //a questo punto mi serve sapere quali sono tutti i plane3D del vertice successivo iterando su enext

    //                                for (HyperGraph::EdgeSet::iterator itnext = enext.begin(); itnext!=enext.end(); itnext++)
    //                                {
    //                                    HyperGraph::Edge* _enext = *itnext;
    //                                    EdgeSE3PlaneSensorCalib * eSE3Calibnext =dynamic_cast< EdgeSE3PlaneSensorCalib*>(_enext);
    //                                    if(eSE3Calibnext)
    //                                    {
    //                                        //configuro l'information matrix anche all'edge successivo
    //                                        eSE3Calibnext->setInformation(info);

    //                                        //estraggo il vertexplane associato a quel determinato EDGE S3 CALIB
    //                                        VertexPlane* vplanenext = dynamic_cast< VertexPlane*>(eSE3Calibnext->vertex(1));
    //                                        sensorOffset = dynamic_cast< VertexSE3*>(eSE3Calibnext->vertex(2));

    //                                        if(vplanenext)
    //                                        {


    //                                            //dal vertexPlane estraggo il Plane3D
    //                                            Plane3D planenext = vplanenext->estimate();

    //                                            //porto il planenext nel frame di riferimento del primo
    //                                            Plane3D modifiedPlaneNext;
    //                                            modifiedPlaneNext=
    //                                                    sensorOffset->estimate().inverse()*
    //                                                    odometry.inverse()*
    //                                                    sensorOffset->estimate()*planenext.toVector();


    //                                            //calcolo errore tra i coeffs dei due piani mappati nello stesso sistema di riferimento
    //                                            Vector4d diff = plane.toVector()-modifiedPlaneNext.toVector();

    //                                            diff.head<3>() *= 10;
    //                                            double error = diff.squaredNorm();

    //                                            if(error<theError) cerr <<"\t (P1) ";

    //                                            else cerr <<"\t [P1] ";
    //                                            cerr
    //                                                    <<setw(12)<<plane.toVector()[0] << " "
    //                                                   <<setw(12)<<plane.toVector()[1] << " "
    //                                                  <<setw(12)<<plane.toVector()[2] << " "
    //                                                 <<setw(12)<<plane.toVector()[3] << " "
    //                                                <<" [P2] "
    //                                               <<setw(12) << modifiedPlaneNext.toVector()[0] << " "
    //                                              <<setw(12)<<modifiedPlaneNext.toVector()[1] << " "
    //                                             <<setw(12)<<modifiedPlaneNext.toVector()[2] << " "
    //                                            <<setw(12)<<modifiedPlaneNext.toVector()[3] << "  "
    //                                            <<setw(20)<<" [ERR] "<< error<<endl;

    //                                            //creo il vettore delle corrispondenze
    //                                            if(error<theError)
    //                                            {

    //                                                EdgePlane* eplane = new EdgePlane;
    //                                                eplane->setVertex(0,const_cast< VertexPlane*>(vplane));
    //                                                eplane->setVertex(1,const_cast< VertexPlane*>(vplanenext));

    //                                                Correspondence corr(eplane,error);

    //                                                mycorrVector.push_back(corr);
    //                                            }


    //                                        }
    //                                    }
    //                                }

    //                            }
    //                        }
    //                    }


    //                    Isometry3d tresult;
    //                    tresult.setIdentity();
    //                    IndexVector iv;
    //                    RansacPlaneLinear ransac;

    //                    ransac.setCorrespondences(mycorrVector);
    //                    ransac.setMaxIterations(2000);
    //                    ransac.setInlierErrorThreshold(1.);
    //                    ransac.setInlierStopFraction(0.5);

    //                    ransac(tresult,iv);

    //                    bool result = testRansac<PlaneMapping, RansacPlaneLinear, EdgePlane>(tresult, mycorrVector,iv);
    //                    cerr << "\t\t planes matching: "<<endl;

    //                    cerr << "adding planes to the output graph..."<<endl;

    //                    std::vector<int> ids;
    //                    std::set<VertexPlane *> murdered;

    //                    for (int i=0;i<iv.size();i++)
    //                    {
    //                        Correspondence tmpCorr=mycorrVector.at(iv.at(i));
    //                        VertexPlane *vp1 = dynamic_cast<VertexPlane*>(tmpCorr.edge()->vertex(0));
    //                        VertexPlane *vp2 = dynamic_cast<VertexPlane*>(tmpCorr.edge()->vertex(1));

    //                        if(vp1 && vp2)
    //                        {
    //                            cerr << "trying..."<<endl;
    //                            bool gottaGo=1;
    //                            for(int j=0;j<ids.size();j++)
    //                            {

    //                                if(ids.at(j)==vp1->id()) gottaGo=0;
    //                                if(ids.at(j)==vp2->id()) gottaGo=0;
    //                            }

    //                            if(gottaGo)
    //                            {
    //                                cerr << "trying to merge vertices... "<<vp2->id() << " is going to be merged on "<<vp1->id()<<" ...";

    //                                vp1->color=Vector3d(1,0,0);
    //                                vp2->color=Vector3d(1,0,0);


    //                                HyperGraph::EdgeSet theSet=vp1->edges();
    //                                for (HyperGraph::EdgeSet::iterator itnext = theSet.begin(); itnext!=theSet.end(); itnext++)
    //                                {
    //                                    HyperGraph::Edge* _enext = *itnext;
    //                                    EdgeSE3PlaneSensorCalib * eSE3Calib =dynamic_cast< EdgeSE3PlaneSensorCalib*>(_enext);
    //                                    if(eSE3Calib)
    //                                    {

    //                                       eSE3Calib->color=Vector3d(1,1,0);
    //                                    }
    //                                 }

    //                                theSet=vp2->edges();
    //                                for (HyperGraph::EdgeSet::iterator itnext = theSet.begin(); itnext!=theSet.end(); itnext++)
    //                                {
    //                                    HyperGraph::Edge* _enext = *itnext;
    //                                    EdgeSE3PlaneSensorCalib * eSE3Calib =dynamic_cast< EdgeSE3PlaneSensorCalib*>(_enext);
    //                                    if(eSE3Calib)
    //                                    {
    //                                       eSE3Calib->color=Vector3d(0,1,0);
    //                                    }
    //                                 }


    //                                bool res=graph.mergeVertices(vp1,vp2,0);
    //                                cerr << "done [ "<<res<<" ]!"<<endl;
    //                                //murdered.insert(vp2);

    //                                ids.push_back(vp1->id());
    //                                ids.push_back(vp2->id());
    //                            }
    //                        }



    //                    }

    ////                    for(std::set<VertexPlane*>::iterator it=murdered.begin();it!=murdered.end();it++)
    ////                    {
    ////                        cerr << "Slaughtering the vertex "<<(*it)->id()<< " has "<< (*it)->edges().size() << endl;
    ////                        graph.removeVertex(*it);


    ////                    }



    //                }



    //            }

    //        }

    //    }

    //    int iteratore=0;
    //    cerr << "removing things..."<<endl;
    //    while(v!=0 && iteratore<2545)
    //    {

    //        OptimizableGraph::Vertex* _v=graph.vertex(iteratore);
    //        v=dynamic_cast<VertexSE3*>(_v);
    //        if(v!=0)
    //        {
    //            if(v->id()!=verticesToBeSaved.at(0) && v->id()!=verticesToBeSaved.at(0) )
    //            graph.removeVertex(v);
    //            //cerr << "vertex "<< iteratore << " is a SE3"<<endl;
    //        }
    //        else
    //            cerr << "vertex "<< iteratore << " is not an S3"<<endl;
    //        iteratore++;

    //    }

    //    cerr << "saving graph..."<<endl;
    //    ofstream oscani ("colorato.g2o");
    //    graph.save(oscani);



    exit(0);

}

